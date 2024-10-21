//
// Created by ou on 2021/12/2.
//

#ifndef PC_UTILS_PARAMETER_H
#define PC_UTILS_PARAMETER_H

#include <any>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/core/demangle.hpp>

using Param = std::any;
using Params = std::map<std::string, Param>;

template<typename ValueType>
inline ValueType any_lexical_cast(const std::any &a) {
    if constexpr(std::is_scalar<ValueType>::value) {
        ValueType result = ValueType();
        if (a.type() == typeid(std::string)
            && boost::conversion::try_lexical_convert(std::any_cast<std::string>(a), result)) {
            return boost::lexical_cast<ValueType>(std::any_cast<std::string>(a));
        } else if (a.type() == typeid(const char *)
                   && boost::conversion::try_lexical_convert(std::any_cast<const char *>(a), result)) {
            return boost::lexical_cast<ValueType>(std::any_cast<const char *>(a));
        }
    }
    return std::any_cast<ValueType>(a);
}


#endif //PC_UTILS_PARAMETER_H
