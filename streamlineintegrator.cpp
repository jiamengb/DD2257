/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, September 19, 2017 - 15:08:33
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/interaction/events/mouseevent.h>
#include <inviwo/core/util/utilities.h>
#include <labstreamlines/integrator.h>
#include <labstreamlines/streamlineintegrator.h>
#include <labutils/scalarvectorfield.h>
#define eucDist2(pos) sqrt(pow(pos.x, 2)+pow(pos.y, 2))

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo StreamlineIntegrator::processorInfo_{
    "org.inviwo.StreamlineIntegrator",  // Class identifier
    "Streamline Integrator",            // Display name
    "KTH Lab",                          // Category
    CodeState::Experimental,            // Code state
    Tags::None,                         // Tags
};

const ProcessorInfo StreamlineIntegrator::getProcessorInfo() const { return processorInfo_; }

StreamlineIntegrator::StreamlineIntegrator()
    : Processor()
    , inData("volIn")
    , meshOut("meshOut")
    , meshBBoxOut("meshBBoxOut")
    , propStartPoint("startPoint", "Start Point", vec2(0.5f, 0.5f), vec2(-1.f), vec2(1.f), vec2(0.1))
    , propSeedMode("seedMode", "Seeds")
    , propNumStepsTaken("numstepstaken", "Number of actual steps", 0, 0, 100000)
    , mouseMoveStart("mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
                     MouseButton::Left, MouseState::Press | MouseState::Move)
// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional),
// increment (optional)); propertyIdentifier cannot have spaces
    , propStepsNum("stepsNum", "Steps Num", 50, 0, 200)
    , propStepSize("stepSize", "Step Size", 0.1, 0.05, 1.0)
    , propMaxArcLen("maxArcLen", "Max Arc Length", 10.0, 0.0, 100.0)
    , propMinSpeed("minSpeed", "Min Speed", 0.1, 0.0, 1.0)
    , propIntegrationDirection("integrationDirection", "Integration Direction")
    , propIntegrationField("integrationField", "Integration Field")
    , propRandomLinesNum("randomLinesNum", "Random Lines Num", 2, 1, 500)
    , propRandomMode("randomMode", "Random Mode")
    , propGridXNum("gridXNum", "Grid-X Num", 2, 1, 25)
    , propGridYNum("gridYNum", "Grid-Y Num", 2, 1, 25)

{
    // Register Ports
    addPort(inData);
    addPort(meshOut);
    addPort(meshBBoxOut);

    // Register Properties
    propSeedMode.addOption("one", "Single Start Point", 0);
    propSeedMode.addOption("multiple", "Multiple Seeds", 1);
    addProperty(propSeedMode);
    addProperty(propStartPoint);
    addProperty(propNumStepsTaken);
    propNumStepsTaken.setReadOnly(true);
    propNumStepsTaken.setSemantics(PropertySemantics::Text);
    addProperty(mouseMoveStart);

    // TODO: Register additional properties
    // addProperty(propertyName);
    addProperty(propStepsNum);
    addProperty(propStepSize);
    addProperty(propMaxArcLen);
    addProperty(propMinSpeed);
    propIntegrationDirection.addOption("forward", "Forward", 0);
    propIntegrationDirection.addOption("backward", "Backward", 1);
    addProperty(propIntegrationDirection);
    propIntegrationField.addOption("vectorField", "Vector Field", 0);
    propIntegrationField.addOption("directionField", "Direction Field", 1);
    addProperty(propIntegrationField);
    propRandomMode.addOption("randomlyInVectorField", "Randomly in Vector Field", 0);
    propRandomMode.addOption("randomlyOnUniformGrid", "Randomly on Uniform Grids", 1);
    propRandomMode.addOption("randomlyMagnitude", "Randomly based on Magnitude", 2);
    addProperty(propRandomMode);
    addProperty(propRandomLinesNum);
    addProperty(propGridXNum);
    addProperty(propGridYNum);

    // Show properties for a single seed and hide properties for multiple seeds
    // (TODO)
    propSeedMode.onChange([this]() {
        if (propSeedMode.get() == 0) {
            util::show(propStartPoint, mouseMoveStart, propNumStepsTaken);
            util::hide(propRandomMode, propRandomLinesNum, propGridXNum, propGridYNum);
        } else {
            util::hide(propStartPoint, mouseMoveStart, propNumStepsTaken);
            util::show(propRandomMode);
        }
    });
    // Task 4
    propRandomMode.onChange([this](){
        // on uniform grid
        if(propRandomMode.get() == 1){
            util::show(propGridXNum, propGridYNum);
            util::hide(propRandomLinesNum);
        }else{
            util::show(propRandomLinesNum);
            util::hide(propGridXNum, propGridYNum);
        }
    });
}

void StreamlineIntegrator::eventMoveStart(Event* event) {
    if (!inData.hasData()) return;
    auto mouseEvent = static_cast<MouseEvent*>(event);
    vec2 mousePos = mouseEvent->posNormalized();

    // Map to bounding box range
    mousePos[0] *= static_cast<float>(BBoxMax_[0] - BBoxMin_[0]);
    mousePos[1] *= static_cast<float>(BBoxMax_[1] - BBoxMin_[1]);
    mousePos += static_cast<vec2>(BBoxMin_);

    // Update starting point
    propStartPoint.set(mousePos);
    event->markAsUsed();
}

void StreamlineIntegrator::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    auto vectorField = VectorField2::createFieldFromVolume(vol);
    BBoxMin_ = vectorField.getBBoxMin();
    BBoxMax_ = vectorField.getBBoxMax();

    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;

    // Make bounding box without vertex duplication, instead of line segments which duplicate
    // vertices, create line segments between each added points with connectivity type of the index
    // buffer
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    Integrator::drawNextPointInPolyline(BBoxMin_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin_[0], BBoxMax_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax_[0], BBoxMin_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    if (propSeedMode.get() == 0) {
        auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
        vec2 startPoint = propStartPoint.get();
        // Draw start point
        Integrator::drawPoint(startPoint, vec4(0, 0, 0, 1), indexBufferPoints.get(), vertices);

        // TODO: Create one stream line from the given start point
        auto indexBufferLines = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);

        // TODO: Use the propNumStepsTaken property to show how many steps have actually been
        // integrated This could be different from the desired number of steps due to stopping
        // conditions (too slow, boundary, ...)
        propNumStepsTaken.set(0);
        
        std::vector<dvec2> streamLine;
        streamLine = createSingleStreamLine(vectorField, startPoint);
        
        int streamLineSize = streamLine.size();
        propNumStepsTaken.set(streamLineSize);
        
        vec4 blue = vec4(0, 0, 1, 1); // Rk4 color
        // vec4 red = vec4(1, 0, 0, 1); // Euler color
        
        for(int i=0; i<streamLineSize; i++){
            Integrator::drawPoint(streamLine[i], blue, indexBufferPoints.get(), vertices);
            Integrator::drawNextPointInPolyline(streamLine[i], blue, indexBufferLines.get(), vertices);
        }
        
    } else {
        // TODO: Seed multiple stream lines either randomly or using a uniform grid
        // (TODO: Bonus, sample randomly according to magnitude of the vector field)
        auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
        auto indexBufferLines = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
        vec4 red = vec4(1, 0, 0, 1); // in vector field
        vec4 green = vec4(0, 1, 0, 1); // on grids
        vec4 cyan = vec4(0, 1, 1, 1); // based on magnitude
        
        int randomLinesNum = propRandomLinesNum.get();
        // on grids
        int gridXNum = propGridXNum.get();
        int gridYNum = propGridYNum.get();
        bool isGrid = false;
        int grid = 0;
        
        switch(propRandomMode.get()){
            // in vector field
            case 0:
                for(int i=0; i<randomLinesNum; i++){
                    dvec2 startPoint = generateRandomStartpoint(vectorField, isGrid, grid);
                    std::vector<dvec2> streamLine = createSingleStreamLine(vectorField, startPoint);
                    for(auto j=streamLine.begin(); j!=streamLine.end(); j++){
                        //LogProcessorInfo("drawVF");
                        Integrator::drawNextPointInPolyline(*j, red, indexBufferLines.get(), vertices);
                        //Integrator::drawPoint(*j, red, indexBufferPoints.get(), vertices);
                    }
                }
                break;
            
            // on grids
            case 1:
                isGrid = true;
                
                for(int i=0; i<gridXNum; i++){
                    dvec2 startPoint;
                    startPoint.x = generateRandomStartpoint(vectorField, isGrid, i+1).x;
                    for(int j=0; j<gridYNum; j++){
                        startPoint.y = generateRandomStartpoint(vectorField, isGrid, j+1).y;
                        std::vector<dvec2> streamLine = createSingleStreamLine(vectorField, startPoint);
                        for(auto k=streamLine.begin(); k!=streamLine.end(); k++){
                            //LogProcessorInfo("drawGrid");
                            Integrator::drawNextPointInPolyline(*k, green, indexBufferLines.get(), vertices);
                            //Integrator::drawPoint(*k, green, indexBufferPoints.get(), vertices);
                        }
                    }
                }
                break;
            
            // based on magnitude
            case 2:
                vec2 vectorFieldNumVertices = vectorField.getNumVerticesPerDim();
                double maxMagnitude = 0.0;
                
                // find maxMagnitude
                for(int i=0; i<vectorFieldNumVertices.x; i++){
                    for(int j=0; j<vectorFieldNumVertices.y; j++){
                        dvec2 tempMag = vectorField.getValueAtVertex({i, j});
                        maxMagnitude = (maxMagnitude<eucDist2(tempMag))? eucDist2(tempMag): maxMagnitude;
                    }
                }
                
                // not suitable if var^2 is too large
                // e.g. for 2x2 [0.01, 0.01, 0.01, 1.0]
                //
                // solution? Prob. Distribution Function
                // p(pt)=eucDist2(pt)/sumEucDist2
                // hash_map<double, dvec2> hashmap;
                // hashmap[int(p(pt)/0.1].add(pt.pos) e.g. 10 buckets
                // PDF: bucketSize for every bucket Bi is its prob. (for this interval), sum(pdf(Bi))=1
                // sort(pdf(B))
                // CDF: cdf(Bi)=sum(pdf(B0)+pdf(B1)+...+pdf(Bi))
                // generate random r in [0, 1], find i thus r in interval i
                // fetch pt in Bi as a seed point
                //
                // question:
                // we treat M*N (2D) as (M*N)*1 (1D) in this solution
                // but are they equivalent?
                for(int i=0; i<randomLinesNum; i++){
                    dvec2 startPoint = generateRandomStartpoint(vectorField, isGrid, grid);
                    double threshold = eucDist2(startPoint)/maxMagnitude;
                    
                    if((double)rand()/RAND_MAX < threshold){
                        std::vector<dvec2> streamLine = createSingleStreamLine(vectorField, startPoint);
                        for(auto j=streamLine.begin(); j!=streamLine.end(); j++){
                            Integrator::drawNextPointInPolyline(*j, cyan, indexBufferLines.get(), vertices);
                            //Integrator::drawPoint(*j, cyan, indexBufferPoints.get(), vertices);
                        }
                    }
                }
                break;
        }
    }
        
    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}  // namespace inviwo

// Create a single stream line
std::vector<dvec2> StreamlineIntegrator::createSingleStreamLine(const VectorField2 &vectorField, const dvec2 &position){
    
    double maxArcLen = propMaxArcLen.get();
    double stepSize = propStepSize.get();
    double minSpeed = propMinSpeed.get();
    int stepsNum = propStepsNum.get();
    bool isBackward = (propIntegrationDirection.get() == 1)? true: false;
    bool isDirectionField = (propIntegrationField.get() == 1)? true: false;

    double arcLen = 0.0;
    double arcLenDiff = 0.0;
    dvec2 pos = position;
    dvec2 nextPos;
    std::vector<dvec2> res;
    
    res.push_back(pos);

    for(int i=0; i<stepsNum; i++){
        nextPos = Integrator::RK4(vectorField, pos, stepSize, isBackward, isDirectionField);
        //LogProcessorInfo("nextPos: (" << nextPos.x << ", " << nextPos.y << ").");
        
        // outside of bbox
        if(vectorField.isInside(nextPos) == false){
            break;
        }
        
        // speed < minSpeed || integration == 0
        arcLenDiff = sqrt(pow(nextPos.x-pos.x, 2)+pow(nextPos.y-pos.y, 2));
        
        if(arcLenDiff/stepSize < minSpeed){
            break;
        }
        
        arcLen += arcLenDiff;
        
        if(arcLen > maxArcLen){
            break;
        }
        
        pos = nextPos;
        res.push_back(pos);
    }
    
    // odd size
    if(res.size()%2!=0){
        res.push_back(pos);
    }
    
    return res;
}

// Generate a random startpoint
dvec2 StreamlineIntegrator::generateRandomStartpoint(const VectorField2 &vectorField, bool isGrid, int grid){
    dvec2 bboxMin = vectorField.getBBoxMin();
    dvec2 bboxMax = vectorField.getBBoxMax();
    int gridXNum = propGridXNum.get();
    int gridYNum = propGridYNum.get();
    dvec2 startPoint;
    
    if(isGrid == true){
        startPoint.x = (bboxMax.x - bboxMin.x)/(double)gridXNum * grid + bboxMin.x;
        startPoint.y = (bboxMax.y - bboxMin.y)/(double)gridYNum * grid + bboxMin.y;
    }else{
        startPoint.x = (bboxMax.x - bboxMin.x) * (double)rand()/RAND_MAX + bboxMin.x;
        startPoint.y = (bboxMax.y - bboxMin.y) * (double)rand()/RAND_MAX + bboxMin.y;
    }
    
    //LogProcessorInfo("startPoint: (" << startPoint.x << ", " << startPoint.y << ").");
    return startPoint;
}

}  // namespace inviwo
