use rocketsim_rs::glam_ext::glam::{Mat3A, Vec3A};

// matching C++ 2.2.1/rs 0.37.0 binds, so no it's not weird
const NORMALIZE_MIN_LENGTH: f32 = f32::EPSILON * f32::EPSILON;
const INVERT_SCALE: Vec3A = Vec3A::new(-1.0, -1.0, 1.0);

#[inline]
#[must_use]
pub fn normalized(vector: Vec3A) -> Vec3A {
    let length = vector.length();

    if length > NORMALIZE_MIN_LENGTH {
        vector / length
    } else {
        Vec3A::ZERO
    }
}

#[inline]
#[must_use]
pub fn to_local(rotation: Mat3A, vector: Vec3A) -> Vec3A {
    rotation.transpose() * vector
}

#[inline]
#[must_use]
pub fn invert_xy(vector: Vec3A) -> Vec3A {
    vector * INVERT_SCALE
}

#[inline]
#[must_use]
pub fn invert_rotation(rotation: Mat3A) -> Mat3A {
    Mat3A::from_cols(
        invert_xy(rotation.x_axis),
        invert_xy(rotation.y_axis),
        invert_xy(rotation.z_axis),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use rocketsim_rs::{
        glam_ext::glam::{EulerRot, Vec3A},
        math::{RotMat, Vec3},
    };

    const TOLERANCE: f32 = 1e-6;

    fn assert_close(left: f32, right: f32) {
        assert!((left - right).abs() < TOLERANCE);
    }

    #[test]
    fn normalization_matches_rocketsim() {
        assert_eq!(normalized(Vec3A::ZERO), Vec3A::ZERO);
        assert!(normalized(Vec3A::ZERO).is_finite());

        let vector = normalized(Vec3A::new(3.0, 4.0, 0.0));
        assert_close(vector.x, 0.6);
        assert_close(vector.y, 0.8);
        assert_close(vector.length(), 1.0);

        let below = Vec3A::new(NORMALIZE_MIN_LENGTH / 2.0, 0.0, 0.0);
        assert_eq!(normalized(below), Vec3A::ZERO);

        let between = Vec3A::new(f32::EPSILON / 2.0, 0.0, 0.0);
        assert!(between.length() > NORMALIZE_MIN_LENGTH);
        assert_eq!(normalized(between), Vec3A::X);
    }

    #[test]
    fn rocketsim_rotation_uses_basis_columns() {
        let rotation = RotMat::new(Vec3::Y, Vec3::new(-1.0, 0.0, 0.0), Vec3::Z).to_glam();

        assert_eq!(rotation.x_axis, Vec3A::Y);
        assert_eq!(rotation.y_axis, Vec3A::NEG_X);
        assert_eq!(rotation.z_axis, Vec3A::Z);
    }

    #[test]
    fn local_coordinates_use_forward_right_up() {
        let rotation = Mat3A::from_cols(Vec3A::Y, Vec3A::NEG_X, Vec3A::Z);
        let vector = rotation.x_axis * 4.0 + rotation.y_axis * -2.0 + rotation.z_axis * 3.0;

        assert_eq!(to_local(Mat3A::IDENTITY, vector), vector);
        assert_eq!(to_local(rotation, vector), Vec3A::new(4.0, -2.0, 3.0));
    }

    #[test]
    fn vector_inversion_round_trips() {
        let vector = Vec3A::new(123.0, -456.0, 789.0);

        assert_eq!(invert_xy(vector), Vec3A::new(-123.0, 456.0, 789.0));
        assert_eq!(invert_xy(invert_xy(vector)), vector);
    }

    #[test]
    fn rotation_inversion_round_trips() {
        assert_eq!(
            invert_rotation(Mat3A::IDENTITY),
            Mat3A::from_cols(Vec3A::NEG_X, Vec3A::NEG_Y, Vec3A::Z)
        );

        let rotation = Mat3A::from_euler(EulerRot::XYZ, 0.3, -1.1, 2.0);
        let inverted = invert_rotation(rotation);

        assert_eq!(invert_rotation(inverted), rotation);
        assert_close(inverted.x_axis.length(), 1.0);
        assert_close(inverted.y_axis.length(), 1.0);
        assert_close(inverted.z_axis.length(), 1.0);
        assert_close(inverted.x_axis.dot(inverted.y_axis), 0.0);
        assert_close(inverted.x_axis.dot(inverted.z_axis), 0.0);
        assert_close(inverted.y_axis.dot(inverted.z_axis), 0.0);
        assert_close(
            inverted.x_axis.cross(inverted.y_axis).dot(inverted.z_axis),
            1.0,
        );
    }
}
