/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Wednesday, September 20, 2017 - 12:04:15
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labstreamlines/integrator.h>
#define eucDist2(pos) sqrt(pow(pos.x, 2)+pow(pos.y, 2))

namespace inviwo {

// TODO: Implement a single integration step here
// Access the vector field with vectorField.interpolate(...)

dvec2 Integrator::Euler(const VectorField2& vectorField, const dvec2& position, const double stepSize, bool isBackward, bool isDirectionField)
{
    dvec2 pos, res;
    
    if(isBackward == true){
        pos = -1.0 * vectorField.interpolate(position);
    }else{
        pos = vectorField.interpolate(position);
    }
    
    // normalize
    if(isDirectionField==true && eucDist2(pos) != 0){
        pos /= eucDist2(pos);
    }
    
    res = position + stepSize * pos;

    return res;
}

dvec2 Integrator::RK4(const VectorField2& vectorField, const dvec2& position, const double stepSize, bool isBackward, bool isDirectionField)
{
    dvec2 k1, k2, k3, k4, res;

    if(isBackward == true){
        k1 = -1.0 * vectorField.interpolate(position);
        k2 = -1.0 * vectorField.interpolate(position + stepSize/2 * k1);
        k3 = -1.0 * vectorField.interpolate(position + stepSize/2 * k2);
        k4 = -1.0 * vectorField.interpolate(position + stepSize * k3);
    }else{
        k1 = vectorField.interpolate(position);
        k2 = vectorField.interpolate(position + stepSize/2 * k1);
        k3 = vectorField.interpolate(position + stepSize/2 * k2);
        k4 = vectorField.interpolate(position + stepSize * k3);
    }
    
    if(isDirectionField == true){
        k1 /= (eucDist2(k1) == 0)? 1.0: eucDist2(k1);
        k2 /= (eucDist2(k2) == 0)? 1.0: eucDist2(k2);
        k3 /= (eucDist2(k3) == 0)? 1.0: eucDist2(k3);
        k4 /= (eucDist2(k4) == 0)? 1.0: eucDist2(k4);
    }

    res = k1/6*stepSize +k2/3*stepSize + k3/3*stepSize + k4/6*stepSize;
    res += position;

    return res;
}

void Integrator::drawPoint(const dvec2& p, const vec4& color, IndexBufferRAM* indexBuffer,
                           std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(p[0], p[1], 0), vec3(0, 0, 1), vec3(p[0], p[1], 0), color});
}

// Alias for draw point
void Integrator::drawNextPointInPolyline(const dvec2& p, const vec4& color,
                                         IndexBufferRAM* indexBuffer,
                                         std::vector<BasicMesh::Vertex>& vertices) {
    Integrator::drawPoint(p, color, indexBuffer, vertices);
}

void Integrator::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                                 IndexBufferRAM* indexBuffer,
                                 std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

}  // namespace inviwo
