/**************************************************************************
* This file is part of PVIO
*
* Copyright (c) ZJU-SenseTime Joint Lab of 3D Vision. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#include <pvio/common.h>
#include <pvio/core/feature_tracker.h>
#include <pvio/core/frontend_worker.h>
#include <pvio/forensics.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/utility/unique_timer.h>

namespace pvio {

PVIO::Core::FeatureTracker::FeatureTracker(PVIO::Core *core, std::shared_ptr<Config> config) :
    core(core), config(config) {
    map = std::make_unique<Map>();
}

PVIO::Core::FeatureTracker::~FeatureTracker() = default;

void PVIO::Core::FeatureTracker::work(std::unique_lock<std::mutex> &l) {
    auto ft_timer = make_timer([](double t) {
        critical_forensics(feature_tracker_time, time) {
            static double avg_time = 0;
            static double avg_count = 0;
            avg_time = (avg_time * avg_count + t) / (avg_count + 1);
            avg_count += 1.0;
            time = avg_time;
        }
    });

    std::unique_ptr<Frame> frame = std::move(frames.front());
    frames.pop_front();
    l.unlock();

    frame->image->preprocess();

    auto [latest_optimized_frame_id, latest_optimized_pose, latest_optimized_motion] = core->frontend->get_latest_state();
    bool is_initialized = latest_optimized_frame_id != nil();
    synchronized(map) {
        if (map->frame_num() > 0) {
            if (is_initialized) {
                size_t latest_optimized_frame_index = map->frame_index_by_id(latest_optimized_frame_id);
                if (latest_optimized_frame_index != nil()) {
                    Frame *latest_optimized_frame = map->get_frame(latest_optimized_frame_index);
                    latest_optimized_frame->pose = latest_optimized_pose;
                    latest_optimized_frame->motion = latest_optimized_motion;
                    for (size_t j = latest_optimized_frame_index + 1; j < map->frame_num(); ++j) {
                        Frame *frame_i = map->get_frame(j - 1);
                        Frame *frame_j = map->get_frame(j);
                        frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, false, false);
                        frame_j->preintegration.predict(frame_i, frame_j);
                    }
                } else {
                    // TODO: unfortunately the frame has slided out, which means we are lost...
                    log_message(PVIO_LOG_WARNING, "SWT cannot catch up.");
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state.reset();
                }
            }
            Frame *last_frame = map->last_frame();
            if (!last_frame->preintegration.data.empty()) {
                if (frame->preintegration.data.empty() || (frame->preintegration.data.front().t - last_frame->image->t > 1.0e-5)) {
                    ImuData imu = last_frame->preintegration.data.back();
                    imu.t = last_frame->image->t;
                    frame->preintegration.data.insert(frame->preintegration.data.begin(), imu);
                }
            }
            frame->preintegration.integrate(frame->image->t, last_frame->motion.bg, last_frame->motion.ba, false, false);
            if (is_initialized) {
                frame->preintegration.predict(last_frame, frame.get());
                std::unique_lock lk(latest_pose_mutex);
                latest_state = {frame->image->t, frame->pose, frame->motion};
                lk.unlock();
            }
            last_frame->track_keypoints(frame.get(), config.get());
        }

        frame->detect_keypoints(config.get());
        map->put_frame(std::move(frame));

        while (map->frame_num() > (is_initialized ? config->feature_tracker_max_frames() : config->feature_tracker_max_init_frames())) {
            map->erase_frame(0);
        }

        forensics(feature_tracker_painter, p) {
            if (p.has_value()) {
                auto painter = std::any_cast<ForensicsPainter *>(p);
                auto frame = map->last_frame();
                painter->set_image(frame->image.get());
                for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                    if (Track *track = frame->get_track(i)) {
                        size_t h = track->id() * 6364136223846793005u + 1442695040888963407;
                        color3b c = {h & 0xFF, (h >> 4) & 0xFF, (h >> 8) & 0xFF};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K).cast<int>(), c, 5);
                        auto it_j = track->keypoint_map().rbegin();
                        auto it_i = std::next(it_j);
                        size_t len = 0;
                        for (; it_i != track->keypoint_map().rend(); ++it_i, ++it_j) {
                            vector<2> pi = apply_k(track->get_keypoint(it_i->first), it_i->first->K);
                            vector<2> pj = apply_k(track->get_keypoint(it_j->first), it_j->first->K);
                            painter->line(pj.cast<int>(), pi.cast<int>(), c, 2);
                            if (len++ > 30) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    core->frontend->issue_frame(map->last_frame());
}

void PVIO::Core::FeatureTracker::track_frame(std::unique_ptr<Frame> frame) {
    auto l = lock();
    frames.emplace_back(std::move(frame));
    resume(l);
}

std::optional<std::tuple<double, PoseState, MotionState>> PVIO::Core::FeatureTracker::get_latest_state() const {
    std::unique_lock lk(latest_pose_mutex);
    return latest_state;
}

} // namespace pvio
