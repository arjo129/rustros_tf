use ndarray::prelude::*;
use nalgebra::*;


fn to_nalgebra_matrix(a: &Array2<f64>) -> Matrix4<f64> {
    Matrix4::new (a[[0,0]], a[[0,1]], a[[0,2]], a[[0,3]],
                a[[1,0]], a[[1,1]], a[[1,2]], a[[1,3]],
                a[[2,0]], a[[2,1]], a[[2,2]], a[[2,3]],
                a[[3,0]], a[[3,1]], a[[3,2]], a[[3,3]])
}


fn log(m: &Matrix4<f64>) -> Matrix4<f64> {
    Matrix4::new(
        m.m11.ln(), 0f64, 0f64, 0f64,
        0f64, m.m22.ln(), 0f64, 0f64,
        0f64, 0f64, m.m33.ln(), 0f64,
        0f64, 0f64, 0f64, m.m44.ln()
    )
}

fn exp(m: &Matrix4<f64>) -> Matrix4<f64> {
    Matrix4::new(
        m.m11.exp(), 0f64, 0f64, 0f64,
        0f64, m.m22.exp(), 0f64, 0f64,
        0f64, 0f64, m.m33.exp(), 0f64,
        0f64, 0f64, 0f64, m.m44.exp()
    )
}


/// Interpolate two se3 matrices
/// 
/// For implementation details see
/// https://www.geometrictools.com/Documentation/InterpolationRigidMotions.pdf
pub fn interpolate(m0: Matrix4<f64>, m1: Matrix4<f64>, weight: f64) -> Matrix4<f64> {
    let geodesic = m1*m0.try_inverse().unwrap();
    let res = linalg::Schur::new(geodesic);
    let (Q,T) = res.unpack();
    let R = weight*log(&T);
    let M = Q*exp(&R)*Q.try_inverse().unwrap();
    M*m0    
}

#[cfg(test)]
mod test{
    use super::*;
    #[test]
    fn test_eigen_values() {
        let m = arr2(
            &[[1f64, 0.0, 0.0, 1.0],
            [0f64, 1.0, 0.0, 1.0],
            [0f64, 0.0, 1.0, 1.0],
            [0f64, 0.0, 0.0, 1.0]]
        );

        let m1 = arr2(
            &[[1f64, 0.0, 0.0, 0.0],
            [0f64, 1.0, 0.0, 0.0],
            [0f64, 0.0, 1.0, 0.0],
            [0f64, 0.0, 0.0, 1.0]]
        );

        let expected = arr2(
            &[[1f64, 0.0, 0.0, 0.5],
            [0f64, 1.0, 0.0, 0.5],
            [0f64, 0.0, 1.0, 0.5],
            [0f64, 0.0, 0.0, 1.0]]
        );

        let m = to_nalgebra_matrix(&m);
        let m1 = to_nalgebra_matrix(&m1);
        let res = interpolate(m, m1, 22.0);
        assert_eq!(res, to_nalgebra_matrix(&expected));
    }
}