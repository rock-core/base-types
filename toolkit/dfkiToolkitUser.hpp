#ifndef dfki_USER_MARSHALLING_HH
#define dfki_USER_MARSHALLING_HH

#include <dfkiToolkitTypes.hpp>

namespace dfki
{
    
    void to_intermediate(DFKI::Quaternion& intermediate, Eigen::Quaterniond const& real_type);
    void from_intermediate(Eigen::Quaterniond& real_type, DFKI::Quaternion& intermediate);

    void to_intermediate(DFKI::Vector3& intermediate, Eigen::Vector3d const& real_type);
    void from_intermediate(Eigen::Vector3d& real_type, DFKI::Vector3& intermediate);

}

#endif

