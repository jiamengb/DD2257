/*********************************************************************
 *  Author  : Anke Friederici
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 **********************************************************************/

#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <labstreamlines/integrator.h>
#include <labutils/scalarvectorfield.h>
#include <labtopo/topology.h>
#include <labtopo/utils/gradients.h>

namespace inviwo {

const vec4 Topology::ColorsCP[6] = {
    vec4(1, 1, 0, 1),    // Saddle - Yellow
    vec4(1, 0, 0, 1),    // AttractingNode - Red
    vec4(0, 0, 1, 1),    // RepellingNode - Blue
    vec4(0.5, 0, 1, 1),  // AttractingFocus - Purple
    vec4(1, 0.5, 0, 1),  // RepellingFocus - Orange
    vec4(0, 1, 0, 1)     // Center - Green
};

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo Topology::processorInfo_{
    "org.inviwo.Topology",    // Class identifier
    "Vector Field Topology",  // Display name
    "KTH Lab",                // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo Topology::getProcessorInfo() const { return processorInfo_; }

Topology::Topology()
    : Processor()
    , inData("inData")
    , outMesh("meshOut")
    , meshBBoxOut("meshBBoxOut")
// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional), increment
// (optional)); propertyIdentifier cannot have spaces
{
    // Register Ports
    addPort(outMesh);
    addPort(inData);
    addPort(meshBBoxOut);

    // TODO: Register additional properties
    // addProperty(propertyName);
}

void Topology::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);

    // Add a bounding box to the mesh
    const dvec2& BBoxMin = vectorField.getBBoxMin();
    const dvec2& BBoxMax = vectorField.getBBoxMax();
    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    Integrator::drawNextPointInPolyline(BBoxMin, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin[0], BBoxMax[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax[0], BBoxMin[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    // Initialize mesh, vertices and index buffers for seperatrices
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;
    // Either add all line segments to this index buffer (one large buffer, two consecutive points
    // make up one line), or use several index buffers with connectivity type strip.
    auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    // auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines,
    // ConnectivityType::Strip);

    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);

    // TODO: Compute the topological skeleton of the input vector field.
    // Find the critical points and color them according to their type.
    // Integrate all separatrices.
    
    std::vector<dvec2> cp = extractCriticalPoints(vectorField);
    
    /*
    size2_t dims = vectorField.getNumVerticesPerDim();

    // Looping through all values in the vector field.
    for (size_t j = 0; j < dims[1]; ++j) {
        for (size_t i = 0; i < dims[0]; ++i) {
            dvec2 vectorValue = vectorField.getValueAtVertex(size2_t(i, j));
        }
    }
    */
    
    

    // Other helpful functions
    // dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));
    // Computing the jacobian at a position
    // dmat2 jacobian = vectorField.derive(pos);
    // Doing the eigen analysis
    // auto eigenResult = util::eigenAnalysis(jacobian);
    // The result of the eigen analysis has attributed eigenvaluesRe eigenvaluesIm and
    // eigenvectors

    // Accessing the colors
    vec4 colorCenter = ColorsCP[static_cast<int>(TypeCP::Center)];
    
    // test
    Integrator::drawPoint(dvec2(3, 3), colorCenter, indexBufferPoints.get(), vertices);
    LogProcessorInfo("draw cp");
    /*
    for(auto iter = cp.begin(); iter!=cp.end(); iter++){
        drawPoint(*iter, colorCenter, indexBufferPoints.get(), vertices);
        LogProcessorInfo("draw cp");
    }
     */

    mesh->addVertices(vertices);
    outMesh.setData(mesh);
}

void Topology::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                               IndexBufferRAM* indexBuffer,
                               std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

std::vector<dvec2> Topology::extractCriticalPoints(const VectorField2 &vectorField)
{
    size2_t dims = vectorField.getNumVerticesPerDim();
    std::vector<dvec2> cp;
    std::vector<std::vector<dvec2>> verticesValue(dims[0], std::vector<dvec2>(dims[1], size2_t(0, 0)));

    // Looping through all values in the vector field.
    for (size_t j = 0; j < dims[1]; ++j) {
        for (size_t i = 0; i < dims[0]; ++i) {
            dvec2 vectorValue = vectorField.getValueAtVertex(size2_t(i, j));
            verticesValue[i][j] = vectorValue;
        }
    }
    
    // change-of-sign test
    for (size_t j = 0; j < dims[1]-1; j++) {
        for (size_t i = 0; i < dims[0]-1; i++) {
            
            // flag. evens for x, odds for y
            std::vector<bool> flag(8, false);
            
            flag[0] = (verticesValue[i][j].x * verticesValue[i][j+1].x < 0)? true: false;
            flag[1] = (verticesValue[i][j].y * verticesValue[i][j+1].y < 0)? true: false;
            flag[2] = (verticesValue[i][j].x * verticesValue[i+1][j].x < 0)? true: false;
            flag[3] = (verticesValue[i][j].y * verticesValue[i+1][j].y < 0)? true: false;
            flag[4] = (verticesValue[i][j+1].x * verticesValue[i+1][j+1].x < 0)? true: false;
            flag[5] = (verticesValue[i][j+1].y * verticesValue[i+1][j+1].y < 0)? true: false;
            flag[6] = (verticesValue[i+1][j].x * verticesValue[i+1][j+1].x < 0)? true: false;
            flag[7] = (verticesValue[i+1][j].y * verticesValue[i+1][j+1].y < 0)? true: false;
            
            // pos_x.size() == 2, pos_y.size() == 2
            // pos_x for finding v_x == 0, pos_y for finding v_y == 0
            std::vector<dvec2> pos_x;
            std::vector<dvec2> pos_y;
            
            // found cp in the cell
            if((flag[0]&&flag[1]) || (flag[2]&&flag[3]) || (flag[4]&&flag[5]) || (flag[6]&&flag[7])){
                
                // find v_x == 0
                // (i, j) - (i, j+1)
                if(flag[0] == true){
                    pos_x.push_back(dvec2(i, j+1.0*(0-verticesValue[i][j].x)/(verticesValue[i][j+1].x-verticesValue[i][j].x)));
                }
                // (i, j) - (i+1, j)
                if(flag[2] == true){
                    pos_x.push_back(dvec2(i+1.0*(0-verticesValue[i][j].x)/(verticesValue[i+1][j].x-verticesValue[i][j].x), j));
                }
                // (i, j+1) - (i+1, j+1)
                if(flag[4] == true){
                    pos_x.push_back(dvec2(i+1.0*(0-verticesValue[i][j+1].x)/(verticesValue[i+1][j+1].x-verticesValue[i][j+1].x), j+1));
                }
                // (i+1, j) - (i+1, j+1)
                if(flag[6] == true){
                    pos_x.push_back(dvec2(i+1, j+1.0*(0-verticesValue[i+1][j].x)/(verticesValue[i+1][j+1].x-verticesValue[i+1][j].x)));
                }
                
                // find v_y == 0
                if(flag[1] == true){
                    pos_y.push_back(dvec2(i, j+1.0*(0-verticesValue[i][j].y)/(verticesValue[i][j+1].y-verticesValue[i][j].y)));
                }
                if(flag[3] == true){
                    pos_y.push_back(dvec2(i+1.0*(0-verticesValue[i][j].y)/(verticesValue[i+1][j].y-verticesValue[i][j].y), j));
                }
                if(flag[5] == true){
                    pos_y.push_back(dvec2(i+1.0*(0-verticesValue[i][j+1].y)/(verticesValue[i+1][j+1].y-verticesValue[i][j+1].y), j+1));
                }
                if(flag[7] == true){
                    pos_y.push_back(dvec2(i+1, j+1.0*(0-verticesValue[i+1][j].y)/(verticesValue[i+1][j+1].y-verticesValue[i+1][j].y)));
                }
                
                LogProcessorInfo("i: "<<i<<"j: "<<j);
                LogProcessorInfo("pos_x:");
                for(auto iter = pos_x.begin(); iter!=pos_x.end(); iter++){
                    LogProcessorInfo(*iter);
                    cp.push_back(*iter);
                }
                LogProcessorInfo("pos_y:");
                for(auto iter = pos_y.begin(); iter!=pos_y.end(); iter++){
                    LogProcessorInfo(*iter);
                    cp.push_back(*iter);
                }
                
                
            }else{
                continue;
            }
            
        }
    }
    
    return cp;
}

}  // namespace inviwo


/*
 // v(x)==0
 if(vectorValue.x == 0 && vectorValue.y == 0){
     dmat2 jacobian = vectorField.derive(size2_t(i, j));
     // det(J) != 0
     if(glm::determinant(jacobian) != 0){
         cp.push_back(size2_t(i, j));
     }
 }
 */
