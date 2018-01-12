#include "light.h"
#include "glPrimitives.h"

class PointLight : public Light
{
public:
    PointLight(const PropertyList &propList)
        : Light(propList.getColor("intensity", Color3f(1.f)))
    {
        m_position = propList.getPoint("position", Point3f::UnitX());
    }

    Vector3f direction(const Point3f& x, float* dist = 0) const
    {
        /// TODO : return normalized direction

        throw SireException("PointLight::direction not implemented yet.");

        return Vector3f(1.0, 0.0, 0.0);
    }

    Color3f intensity(const Point3f& x) const
    {
        /// TODO : return intensity

        throw SireException("PointLight::intensity not implemented yet.");

        return Color3f(0.f);
    }

    void draw()
    {
        if(m_shader){
            m_shader->bind();
            Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
            glUniformMatrix4fv(m_shader->uniform("mat_obj"), 1, GL_FALSE, M.data());
            glUniform3fv(m_shader->uniform("color"), 1, (m_intensity/m_intensity.maxCoeff()).eval().data());
            Point::draw(m_shader, m_position);
        }
    }

    std::string toString() const {
        return tfm::format(
            "PointLight[\n"
            "  intensity = %s\n"
            "  position = %s\n"
            "]", m_intensity.toString(),
                 ::toString(m_position));
    }

protected:
    Point3f m_position;
};

REGISTER_CLASS(PointLight, "pointLight")
