#include "lidar.hpp"
#include "chromatic/core/helpers.hpp"
#include "config.hpp"

namespace chromatic {

    void run_dresets(ms pollrate) {
        while (comp_state != CompState::DISABLE) {
            front_lidar.update();
            left_lidar.update();
            right_lidar.update();
            delay_for(pollrate);
        }
    }

    // checks to see if a pose is currently aiming at an object, true if true
    bool check_collision(Pose pose) {
        for (auto& object : elements) {
            if (object.projection_collision(pose)) return true;
        }
        return false;
    }

    bool side_reset(Quad quadrant) {
        Vec position = localizer->get_pose().pos;
        double angle_rad = localizer->get_pose().dir;

        Vec reset_pos = position;
        double reset_angle = to_deg(angle_rad);

        Pose left_pose = Pose{reset_pos + rotate(Vec{-left_lidar.get_offset(), 0}, angle_rad), wrap_angle(reset_angle + 90, false)};
        Pose right_pose = Pose{reset_pos + rotate(Vec{-right_lidar.get_offset(), 0}, angle_rad), wrap_angle(reset_angle - 90, false)};

        double offset_angle = fmod(reset_angle + 45, 90) - 45;

        auto safe_left_norm = [&] {
            double measurement = left_lidar.find_norm_distance(offset_angle);
            if (check_collision(left_pose) || measurement < 0) return -1.0;
            return measurement;
        };

        auto safe_right_norm = [&] {
            double measurement = right_lidar.find_norm_distance(offset_angle);
            if (check_collision(right_pose) || measurement < 0) return -1.0;
            return measurement;
        };

        double m_l = safe_left_norm();
        double m_r = safe_right_norm();

        if (within_bounds_periodic(315, 45, reset_angle)) {
            switch (quadrant) {
            case Quad::Q1: case Quad::Q2: if (m_l > 0) {reset_pos.y = walls.top - m_l;} break;
            case Quad::Q3: case Quad::Q4: if (m_r > 0) {reset_pos.y = -walls.bottom + m_r;} break;
            }
        } else if (within_bounds_periodic(135, 225, reset_angle)) {
            switch (quadrant) {
            case Quad::Q1: case Quad::Q2: if (m_r > 0) {reset_pos.y = walls.top - m_r;} break;
            case Quad::Q3: case Quad::Q4: if (m_l > 0) {reset_pos.y = -walls.bottom + m_l;} break;
            }
        } else if (within_bounds_periodic(45, 135, reset_angle)) {
            switch (quadrant) {
            case Quad::Q1: case Quad::Q4: if (m_r > 0) {reset_pos.x = walls.right - m_r;} break;
            case Quad::Q2: case Quad::Q3: if (m_l > 0) {reset_pos.x = -walls.left + m_l;} break;
            }
        } else if (within_bounds_periodic(225, 315, reset_angle)) {
            switch (quadrant) {
            case Quad::Q1: case Quad::Q4: if (m_l > 0) {reset_pos.x = walls.right - m_l;} break;
            case Quad::Q2: case Quad::Q3: if (m_r > 0) {reset_pos.x = -walls.left + m_r;} break;
            }
        }

        localizer->set_pose(Pose{reset_pos, angle_rad});
        return true;
    }

    // as this uses two sensors
    bool fwd_reset(Quad quadrant, bool calculate_heading = false) {
        Vec position = localizer->get_pose().pos;
        double angle_rad = localizer->get_pose().dir;

        Vec reset_pos = localizer->get_pose().pos;
        double reset_angle = to_deg(angle_rad);

        Pose r_pose = Pose{reset_pos + rotate(Vec{0, front_lidar.get_left_offset()}, angle_rad)};
        Pose l_pose = Pose{reset_pos + rotate(Vec{0, -front_lidar.get_right_offset()}, angle_rad)};

        if (check_collision(r_pose) || check_collision(l_pose)) return false;

        double offset = front_lidar.find_offset_deg_ccw();
        if (offset < 0) return false;

        double closest_axial = static_cast<int>(fmod(reset_angle + 45, 360) / 90) * 90;
        reset_angle = calculate_heading ? offset + closest_axial : reset_angle;

        double measurement = front_lidar.find_norm_distance(reset_angle - closest_axial);
        if (measurement < 0) return false;

        // there are actually only 8 situations where front reset is used
        if (within_bounds_periodic(315, 45, reset_angle) && (quadrant == Quad::Q1 || quadrant == Quad::Q4)) {
            reset_pos.x = walls.right - measurement;
        } else if (within_bounds_periodic(45, 135, reset_angle) && (quadrant == Quad::Q1 || quadrant == Quad::Q2)) {
            reset_pos.y = walls.top - measurement;
        } else if (within_bounds_periodic(135, 225, reset_angle) && (quadrant == Quad::Q2 || quadrant == Quad::Q3)) {
            reset_pos.x = -walls.left + measurement;
        } else if (within_bounds_periodic(225, 315, reset_angle) && (quadrant == Quad::Q3 || quadrant == Quad::Q4)) {
            reset_pos.y = -walls.bottom + measurement;
        }

        localizer->set_pose(Pose{reset_pos, reset_angle});

        return true;
    }

}
