//
// Created by max_3 on 25-5-13.
//

#ifndef AR1LINEARAA_HPP
#define AR1LINEARAA_HPP
#include <vector>
#include <memory>
class AR1Linear {
public:
    virtual ~AR1Linear() = default;
    virtual std::vector<double> sensor2Angle(std::vector<double> inputSensorData, std::vector<double> preSensorData) = 0;
};

#define ExportAR1Linear(x)        \
namespace AR1LinearImpl {          \
    static_assert(std::is_base_of_v<AR1Linear, x>,        \
    "Class must inherit from AR1Linear publicly");    \
    std::unique_ptr<AR1Linear> x##_create() {              \
        return std::unique_ptr<AR1Linear>(reinterpret_cast<AR1Linear*>(new x()));       \
    }                                                     \
    std::unique_ptr<AR1Linear>& x##_instance() {                      \
        static auto instance = x##_create();              \
        return instance;                                   \
    }  \
}

#define ImportAR1Linear(x)           \
namespace AR1LinearImpl {                                  \
    extern std::unique_ptr<AR1Linear>& x##_instance();                       \
}                       \
inline std::unique_ptr<AR1Linear>& x() { \
  return AR1LinearImpl::x##_instance(); \
}

#endif //AR1LINEARAA_HPP
