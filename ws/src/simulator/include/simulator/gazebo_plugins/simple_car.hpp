
#pragma once

#include <gazebo/common/Plugin.hh>

#include <memory>


namespace gazebo_plugins
{
class SimpleCarPrivate;

class SimpleCar : public gazebo::ModelPlugin
{
public:
    SimpleCar();

    ~SimpleCar();
protected:
    void Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p) override;
    
    void Reset() override;
    
private:
    std::unique_ptr<SimpleCarPrivate> impl_p;
};
}