#ifndef INCLUDE_OKVIS_LIDARMAPPING_CONFIG_IMPL_HPP
#define INCLUDE_OKVIS_LIDARMAPPING_CONFIG_IMPL_HPP

#include "se/common/yaml.hpp"

namespace se {


template<typename DataConfigT, typename SensorConfigT>
Config<DataConfigT, SensorConfigT>::Config()
{
}



template<typename DataConfigT, typename SensorConfigT>
Config<DataConfigT, SensorConfigT>::Config(const std::string& yaml_file) :
        map(yaml_file), data(yaml_file)
{
    sensor.readYaml(yaml_file);
}



template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const Config<DataConfigT, SensorConfigT>& c)
{
    os << "Data config -----------------------\n";
    os << c.data;
    os << "Map config ------------------------\n";
    os << c.map;
    os << "Sensor config ---------------------\n";
    os << c.sensor;
    return os;
}


template<typename DataConfigT, typename SensorConfigT>
OkvisSubmapsConfig<DataConfigT, SensorConfigT>::OkvisSubmapsConfig()
{
}



template<typename DataConfigT, typename SensorConfigT>
OkvisSubmapsConfig<DataConfigT, SensorConfigT>::OkvisSubmapsConfig(const std::string& yaml_file) :
        general(yaml_file), se_config(yaml_file)
{
}



template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const OkvisSubmapsConfig<DataConfigT, SensorConfigT>& c)
{
  os << "General submap config -----------------------\n";
  os << c.general;
  os << "SE2 config -----------------------\n";
  os << c.se_config;
  return os;
}


} // namespace se

#endif // INCLUDE_OKVIS_LIDARMAPPING_CONFIG_IMPL_HPP
