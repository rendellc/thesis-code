#pragma once

#include <gazebo/common/Plugin.hh>

#include <memory>


namespace gazebo_plugins
{
class WheelPrivate;

class Wheel : public gazebo::ModelPlugin
{
public:
    Wheel();

    ~Wheel();
protected:
    void Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p) override;
    
    void Reset() override;
    
private:
    std::unique_ptr<WheelPrivate> impl_p;
};
}