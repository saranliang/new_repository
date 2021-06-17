/**
 * @file dualquat/dualquat_base.h
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dualquat             //定义命名空间
{

template<typename T>     //定义对偶四元数类的空没
class DualQuaternion;

using DualQuaternionf = DualQuaternion<float>;
using DualQuaterniond = DualQuaternion<double>;

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Quaternion = Eigen::Quaternion<T>;    //使用的四元数就是Eigen只的

template<typename T>
class DualQuaternion
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");
public:
    using value_type = T;

/* Constructors */
    DualQuaternion(){}

    DualQuaternion(
        const Quaternion<T>& real,     
        const Quaternion<T>& dual)
        : real_(real),
          dual_(dual)
    {}

    /**
     * Creates a new dual quaternion with the given vector.   --利用数据给出对偶四元数
     */
    explicit DualQuaternion(const Vector3<T>& v)
        : DualQuaternion(
            Quaternion<T>::Identity(),
            Quaternion<T>(T(0), v.x(), v.y(), v.z()))
    {}

    /**
     * Creates a new dual quaternion with the given line (in Plucker coordinates).
     */
    DualQuaternion(const Vector3<T>& l, const Vector3<T>& m)
        : DualQuaternion(
            Quaternion<T>(T(0), l.x(), l.y(), l.z()),
            Quaternion<T>(T(0), m.x(), m.y(), m.z()))
    {}

/* Accessors      ---操作*/
    const Quaternion<T>& real() const noexcept { return real_; }
    const Quaternion<T>& dual() const noexcept { return dual_; }

    Quaternion<T>& real() noexcept { return real_; }
    Quaternion<T>& dual() noexcept { return dual_; }

/* Assignment operators */
    DualQuaternion& operator += (const DualQuaternion&);
    DualQuaternion& operator -= (const DualQuaternion&);
    DualQuaternion& operator *= (const DualQuaternion&);
    DualQuaternion& operator *= (T);

private:
    static constexpr bool needs_to_align = (sizeof(Quaternion<T>) % 16) == 0;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(needs_to_align)

private:
    Quaternion<T> real_;
    Quaternion<T> dual_;
};

/* Assignment operators */
//coeffs的顺序是(x,y,z,w),也是内存中的存储结构
//四元数的定义是(w,x,y,z)
template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator += (const DualQuaternion& rhs)
{
    real_ = real_.coeffs() + rhs.real().coeffs();     
    dual_ = dual_.coeffs() + rhs.dual().coeffs();
    return *this;
}

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator -= (const DualQuaternion& rhs)
{
    real_ = real_.coeffs() - rhs.real().coeffs();
    dual_ = dual_.coeffs() - rhs.dual().coeffs();
    return *this;
}

//this = this*rhs  
template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator *= (const DualQuaternion& rhs)
{
    auto temp = real_;
    real_ = temp * rhs.real();
    dual_ = (temp * rhs.dual()).coeffs() + (dual_ * rhs.real()).coeffs();    
    return *this;
}

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator *= (T rhs)
{
    real_.coeffs() *= rhs;
    dual_.coeffs() *= rhs;
    return *this;
}

/* Unary operators */

template<typename T>
DualQuaternion<T>
operator + (const DualQuaternion<T>& rhs)
{
    return rhs;
}

template<typename T>
DualQuaternion<T>
operator - (const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp;
    temp.real() = -rhs.real().coeffs();
    temp.dual() = -rhs.dual().coeffs();
    return temp;
}

/* Binary operators */

template<typename T>
DualQuaternion<T>
operator + (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp += rhs;
}

template<typename T>
DualQuaternion<T>
operator - (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp -= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp *= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (const DualQuaternion<T>& lhs, T rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp *= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (T lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(rhs);
    return temp *= lhs;
}

}   // namespace dualquat
