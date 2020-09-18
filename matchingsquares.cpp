/*********************************************************************
 *  Author  : Himangshu Saikia, Wiebke Koepp, Anke Friederici
 *  Init    : Monday, September 11, 2017 - 12:58:42
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labmarchingsquares/marchingsquares.h>
#include <inviwo/core/util/utilities.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MarchingSquares::processorInfo_{
    "org.inviwo.MarchingSquares",  // Class identifier
    "Marching Squares",            // Display name
    "KTH Labs",                    // Category
    CodeState::Experimental,       // Code state
    Tags::None,                    // Tags
};

const ProcessorInfo MarchingSquares::getProcessorInfo() const { return processorInfo_; }

MarchingSquares::MarchingSquares()
    : Processor()
    , inData("volumeIn")
    , meshOut("meshOut")
    , propShowGrid("showGrid", "Show Grid")
    , propGridColor("gridColor", "Grid Lines Color", vec4(0.0f, 0.0f, 0.0f, 1.0f), vec4(0.0f),
                    vec4(1.0f), vec4(0.1f), InvalidationLevel::InvalidOutput,
                    PropertySemantics::Color)
    , propDeciderType("deciderType", "Decider Type")
    , propMultiple("multiple", "Iso Levels")
    , propIsoValue("isovalue", "Iso Value")
    , propIsoColor("isoColor", "Color", vec4(0.0f, 0.0f, 1.0f, 1.0f), vec4(0.0f), vec4(1.0f),
                   vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , propNumContours("numContours", "Number of Contours", 1, 1, 50, 1)
    , propIsoTransferFunc("isoTransferFunc", "Colors", &inData) {
    // Register ports
    addPort(inData);
    addPort(meshOut);

    // Register properties
    addProperty(propShowGrid);
    addProperty(propGridColor);

    addProperty(propDeciderType);
    propDeciderType.addOption("midpoint", "Mid Point", 0);
    propDeciderType.addOption("asymptotic", "Asymptotic", 1);

    addProperty(propMultiple);

    propMultiple.addOption("single", "Single", 0);
    addProperty(propIsoValue);
    addProperty(propIsoColor);

    propMultiple.addOption("multiple", "Multiple", 1);
    addProperty(propNumContours);
    addProperty(propIsoTransferFunc);

    // The default transfer function has just two blue points
    propIsoTransferFunc.get().clear();
    propIsoTransferFunc.get().add(0.0f, vec4(0.0f, 0.0f, 1.0f, 1.0f));
    propIsoTransferFunc.get().add(1.0f, vec4(0.0f, 0.0f, 1.0f, 1.0f));
    propIsoTransferFunc.setCurrentStateAsDefault();

    util::hide(propGridColor, propNumContours, propIsoTransferFunc);

    // Show the grid color property only if grid is actually displayed
    propShowGrid.onChange([this]() {
        if (propShowGrid.get()) {
            util::show(propGridColor);
        } else {
            util::hide(propGridColor);
        }
    });

    // Show options based on display of one or multiple iso contours
    propMultiple.onChange([this]() {
        if (propMultiple.get() == 0) {
            util::show(propIsoValue, propIsoColor);
            util::hide(propNumContours, propIsoTransferFunc);
        } else {
            util::hide(propIsoValue);
            util::show(propIsoColor, propNumContours);

            // TODO (Bonus): Comment out above if you are using the transfer function
            // and comment in below instead
            // util::hide(propIsoValue, propIsoColor);
            // util::show(propNumContours, propIsoTransferFunc);
        }
    });
}

void MarchingSquares::process() {
    if (!inData.hasData()) {
        return;
    }

    // Create a structured grid from the input volume
    auto vol = inData.getData();
    auto grid = ScalarField2::createFieldFromVolume(vol);

    // Extract the minimum and maximum value from the input data
    const double minValue = grid.getMinValue();
    const double maxValue = grid.getMaxValue();

    // Set the range for the isovalue to that minimum and maximum
    propIsoValue.setMinValue(minValue);
    propIsoValue.setMaxValue(maxValue);

    // You can print to the Inviwo console with Log-commands:
    LogProcessorInfo("This scalar field contains values between " << minValue << " and " << maxValue
                                                                  << ".");
    // You can also inform about errors and warnings:
    // LogProcessorWarn("I am warning about something"); // Will print warning message in yellow
    // LogProcessorError("I am letting you know about an error"); // Will print error message in red
    // (There is also LogNetwork...() and just Log...(), these display a different source,
    // LogProcessor...() for example displays the name of the processor in the workspace while
    // Log...() displays the identifier of the processor (thus with multiple processors of the
    // same kind you would not know which one the information is coming from

    // Get the definition of our structured grid with
    // - number of vertices in each dimension {nx, ny}
    const ivec2 nVertPerDim = grid.getNumVerticesPerDim();
    // - bounding box {xmin, ymin} - {xmax, ymax}
    const dvec2 bBoxMin = grid.getBBoxMin();
    const dvec2 bBoxMax = grid.getBBoxMax();
    // - cell size {dx, dy}
    const dvec2 cellSize = grid.getCellSize();

    // Values at the vertex positions can be accessed by the indices of the vertex
    // with index i ranging between [0, nx-1] and j in [0, ny-1]
    ivec2 ij = {0, 0};
    double  valueAt00= grid.getValueAtVertex(ij);
    LogProcessorInfo("The value at (0,0) is: " << valueAt00 << ".");

    // Initialize the output: mesh and vertices
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    // Properties are accessed with propertyName.get()
    if (propShowGrid.get()) {
        // TODO: Add grid lines of the given color
        // row
        for(auto j = 0; j < nVertPerDim[1]; j++){
            for(auto i = 0; i < nVertPerDim[0]-1; i++){
                auto indexBufferGrid = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
                // vindex={i, j}
                // vx=xmin+cellsize_x*i
                // vy=ymin+cellsize_y*j
                vec2 v1 = vec2(
                    bBoxMin[0]+cellSize[0]*(float)i,
                    bBoxMin[1]+cellSize[1]*(float)j
                    );
                vec2 v2 = vec2(
                    bBoxMin[0]+cellSize[0]*(float)(i+1),
                    bBoxMin[1]+cellSize[1]*(float)j
                    );
                drawLineSegment(v1, v2, propGridColor.get(), indexBufferGrid.get(), vertices);
            }
        }

        // column
        for(auto i = 0; i < nVertPerDim[0]; i++){
            for(auto j = 0; j < nVertPerDim[1]-1; j++){
                auto indexBufferGrid = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
                vec2 v1 = vec2(
                    bBoxMin[0]+cellSize[0]*(float)i,
                    bBoxMin[1]+cellSize[1]*(float)j
                    );
                vec2 v2 = vec2(
                    bBoxMin[0]+cellSize[0]*(float)i,
                    bBoxMin[1]+cellSize[1]*(float)(j+1)
                    );
                drawLineSegment(v1, v2, propGridColor.get(), indexBufferGrid.get(), vertices);
            }
        }

        // The function drawLineSegments creates two vertices at the specified positions,
        // that are placed into the Vertex vector defining our mesh.
        // An index buffer specifies which of those vertices should be grouped into to make up
        // lines/trianges/quads. Here two vertices make up a line segment.
        // auto indexBufferGrid = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

        // Draw a line segment from v1 to v2 with a the given color for the grid
        // vec2 v1 = vec2(0.5, 0.5);
        // vec2 v2 = vec2(0.7, 0.7);
        // drawLineSegment(v1, v2, propGridColor.get(), indexBufferGrid.get(), vertices);
    }

    // Iso contours

    // TODO (Bonus) Gaussian filter
    // Our input is const (i.e. cannot be altered), but you need to compute smoothed data and write
    // it somewhere
    // Create an editable structured grid with ScalarField2 smoothedField =
    // ScalarField2(nVertPerDim, bBoxMin, bBoxMax - bBoxMin); Values can be set with
    // smoothedField.setValueAtVertex({0, 0}, 4.2);
    // and read again in the same way as before
    // smoothedField.getValueAtVertex(ij);

    if (propMultiple.get() == 0) {
        // TODO: Draw a single isoline at the specified isovalue (propIsoValue)
        // and color it with the specified color (propIsoColor)
        double isoVal = propIsoValue.get();
        std::vector<vec2> verts;

        // Left-Top, Right-Top, Right-Bottom, Left-Bottom (clockwise)

        for(auto j = 0; j < nVertPerDim[1]-1; j++){
            for(auto i = 0; i < nVertPerDim[0]-1; i++){
                ivec2 ij = {i, j};
                std::vector<double> vertVals;
                
                vertVals.push_back(grid.getValueAtVertex({i, j+1}));
                vertVals.push_back(grid.getValueAtVertex({i+1, j+1}));
                vertVals.push_back(grid.getValueAtVertex({i+1, j}));
                vertVals.push_back(grid.getValueAtVertex({i, j}));

                auto minmax = std::minmax({vertVals[0], vertVals[1], vertVals[2], vertVals[3]});
                double minVal = minmax.first;
                double maxVal = minmax.second;

                // midpoint & asympoint
                //if(propDeciderType.get() == 0){
                    if(maxVal >= isoVal && minVal < isoVal){
                        verts = Core(isoVal, ij, vertVals, bBoxMin, cellSize);
                    }
                    
                    for(auto k = 0; k < verts.size()-1; k=k+2){
                        auto indexBufferGrid = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
                        drawLineSegment(verts[k], verts[k+1], propIsoColor.get(), indexBufferGrid.get(), vertices);
                        LogProcessorInfo("v[k]: " << verts[k][0] << "," << verts[k][1] << ".");
                        LogProcessorInfo("v[k+1]: " << verts[k+1][0] << "," << verts[k+1][1] << ".");
                    }
                //}
            }
        }
    }
    // propMultiple.get() == 1 <- several isolines
    else {
        // TODO: Draw the given number (propNumContours) of isolines between
        // the minimum and maximum value
        // TODO (Bonus): Use the transfer function property to assign a color
        // The transfer function normalizes the input data and sampling colors
        // from the transfer function assumes normalized input, that means
        // vec4 color = propIsoTransferFunc.get().sample(0.0f);
        // is the color for the minimum value in the data
        // vec4 color = propIsoTransferFunc.get().sample(1.0f);
        // is the color for the maximum value in the data
        
        int n = propNumContours.get();
        double diff = (double)(maxValue-minValue)/(n+1);
        std::vector<double> isoVals;
        std::vector<vec4> isoColors;
        
        for(auto i = 0; i < n; i++){
            isoVals.push_back((double)(minValue + diff*(i+1)));
            LogProcessorInfo("isoVal: "<<isoVals.back()<<".");
            // assign colors
            isoColors.push_back(propIsoTransferFunc.get().sample((double)(diff*(i+1)/(maxValue-minValue))));
            LogProcessorInfo("isoColor: "<<isoColors.back()<<".");
        }

        // same as propMultiple.get() == 0
        for(auto m = 0; m < n; m++){
            double isoVal = isoVals[m];
            LogProcessorInfo("isoVal: "<<isoVal<<".");
            std::vector<vec2> verts;

            // Left-Top, Right-Top, Right-Bottom, Left-Bottom (clockwise)

            for(auto j = 0; j < nVertPerDim[1]-1; j++){
                for(auto i = 0; i < nVertPerDim[0]-1; i++){
                    ivec2 ij = {i, j};
                    std::vector<double> vertVals;
                    
                    vertVals.push_back(grid.getValueAtVertex({i, j+1}));
                    vertVals.push_back(grid.getValueAtVertex({i+1, j+1}));
                    vertVals.push_back(grid.getValueAtVertex({i+1, j}));
                    vertVals.push_back(grid.getValueAtVertex({i, j}));

                    auto minmax = std::minmax({vertVals[0], vertVals[1], vertVals[2], vertVals[3]});
                    double minVal = minmax.first;
                    double maxVal = minmax.second;

                    if(maxVal >= isoVal && minVal < isoVal){
                        verts = Core(isoVal, ij, vertVals, bBoxMin, cellSize);
                    }
                        
                    for(auto k = 0; k < verts.size()-1; k=k+2){
                        auto indexBufferGrid = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
                        drawLineSegment(verts[k], verts[k+1], propIsoColor.get(), indexBufferGrid.get(), vertices);
                        LogProcessorInfo("v[k]: " << verts[k][0] << "," << verts[k][1] << ".");
                        LogProcessorInfo("v[k+1]: " << verts[k+1][0] << "," << verts[k+1][1] << ".");
                    }
                }
            }
        }
    }

    // Note: It is possible to add multiple index buffers to the same mesh,
    // thus you could for example add one for the grid lines and one for
    // each isoline
    // Also, consider to write helper functions to avoid code duplication
    // e.g. for the computation of a single iso contour

    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}

void MarchingSquares::drawLineSegment(const vec2& v1, const vec2& v2, const vec4& color,
                                      IndexBufferRAM* indexBuffer,
                                      std::vector<BasicMesh::Vertex>& vertices) {
    // Add first vertex
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    // A vertex has a position, a normal, a texture coordinate and a color
    // we do not use normal or texture coordinate, but still have to specify them
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    // Add second vertex
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

// 16 cases of the square
// actually 12 cases
// 1111, 0000 <- no contour
// 1110/0001, 1101/0010, 1011/0100, 0111/1000 <- 1 slash
// 1100, 1001, 0011, 0110 <- vertical/horizontial
// 1010*2, 0101*2 <- 2 parallel slashes
std::vector<vec2> MarchingSquares::Core(double isoVal, ivec2 ij, std::vector<double> vertVals, dvec2 bBoxMin, dvec2 cellSize){
    // return endpoints of the contour line
    std::vector<vec2> cEndpoints;
    // posLeft, posTop, posRight, posBottom (clockwise)
    // pos_x = xmin + cellsize_x*i + cellsize_x*interpolationRatio
    // pos_y = ymin + cellsize_y*j + cellsize_y*interpolationRatio
    // 0<=i<nx-1, 0<=j<ny-1

    // vertVals: Left-top[0] -> Right-top[1] -> Right-bottom[2] -> Left-bottom[3]
    vec2 posLeft = vec2(
        bBoxMin[0] + cellSize[0]*ij[0],
        bBoxMin[1] + cellSize[1]*ij[1] + cellSize[1]*((float)(isoVal-vertVals[3])/(vertVals[0]-vertVals[3]))
        );
    vec2 posTop = vec2(
        bBoxMin[0] + cellSize[0]*ij[0] + cellSize[0]*((float)(isoVal-vertVals[0])/(vertVals[1]-vertVals[0])),
        bBoxMin[1] + cellSize[1]*(ij[1]+1)
        );
    vec2 posRight = vec2(
        bBoxMin[0] + cellSize[0]*(ij[0]+1),
        bBoxMin[1] + cellSize[1]*ij[1] + cellSize[1]*((float)(isoVal-vertVals[2])/(vertVals[1]-vertVals[2]))
        );
    vec2 posBottom = vec2(
        bBoxMin[0] + cellSize[0]*ij[0] + cellSize[0]*((float)(isoVal-vertVals[3])/(vertVals[2]-vertVals[3])),
        bBoxMin[1] + cellSize[1]*ij[1]
        );
    
    double midpointVal = std::accumulate(vertVals.begin(), vertVals.end(), 0.0)/vertVals.size();
    double asympointVal = (vertVals[1]*vertVals[3]-vertVals[0]*vertVals[2])/(vertVals[1]+vertVals[3]-vertVals[0]-vertVals[2]);

    // flags
    bool isLT0 = vertVals[0] < isoVal ? true : false;
    bool isRT0 = vertVals[1] < isoVal ? true : false;
    bool isRB0 = vertVals[2] < isoVal ? true : false;
    bool isLB0 = vertVals[3] < isoVal ? true : false;
    bool isMid0 = midpointVal < isoVal ? true : false;
    bool isAsym0 = asympointVal < isoVal ? true : false;

    // case 1111/0000
    // no contour

    // case 1110/0001
    if((!isLT0 && !isRT0 && !isRB0 && isLB0) || (isLT0 && isRT0 && isRB0 && !isLB0)){
        cEndpoints.push_back(posLeft);
        cEndpoints.push_back(posTop);
        LogProcessorInfo("Left, Top");
    }

    // case 1101/0010
    else if((!isLT0 && !isRT0 && isRB0 && !isLB0) || (isLT0 && isRT0 && !isRB0 && isLB0)){
        cEndpoints.push_back(posRight);
        cEndpoints.push_back(posBottom);
        LogProcessorInfo("Right, Bottom");
    }

    // case 1011/0100
    else if((!isLT0 && isRT0 && !isRB0 && !isLB0) || (isLT0 && !isRT0 && isRB0 && isLB0)){
        cEndpoints.push_back(posRight);
        cEndpoints.push_back(posTop);
        LogProcessorInfo("Right, Top");
    }

    // case 0111/1000
    else if((isLT0 && !isRT0 && !isRB0 && !isLB0) || (!isLT0 && isRT0 && isRB0 && isLB0)){
        cEndpoints.push_back(posLeft);
        cEndpoints.push_back(posBottom);
        LogProcessorInfo("Left, Bottom");
    }

    // case 1100/0011
    else if((!isLT0 && !isRT0 && isRB0 && isLB0) || (isLT0 && isRT0 && !isRB0 && !isLB0)){
        cEndpoints.push_back(posLeft);
        cEndpoints.push_back(posRight);
        LogProcessorInfo("Left, Right");
    }

    // case 1001/0110
    else if((!isLT0 && isRT0 && isRB0 && !isLB0) || (isLT0 && !isRT0 && !isRB0 && isLB0)){
        cEndpoints.push_back(posTop);
        cEndpoints.push_back(posBottom);
        LogProcessorInfo("Top, Bottom");
    }

    // midpoint
    else if(propDeciderType.get() == 0){
        // case 1010 & 0101 
        // (a)
        if((!isLT0 && isRT0 && !isRB0 && isLB0 && isMid0) || (isLT0 && !isRT0 && isRB0 && !isLB0 && !isMid0)){
            cEndpoints.push_back(posLeft);
            cEndpoints.push_back(posTop);
            cEndpoints.push_back(posRight);
            cEndpoints.push_back(posBottom);
            LogProcessorInfo("Mid, Left, Top, Right, Bottom");
        }
        // (b)
        else if((!isLT0 && isRT0 && !isRB0 && isLB0 && !isMid0) || (isLT0 && !isRT0 && isRB0 && !isLB0 && isMid0)){
            cEndpoints.push_back(posRight);
            cEndpoints.push_back(posTop);
            cEndpoints.push_back(posLeft);
            cEndpoints.push_back(posBottom);
            LogProcessorInfo("Mid, Right, Top, Left, Bottom");
        }
        else;
    }

    // asym
    else if(propDeciderType.get() == 1){
        // case 1010 & 0101 
        // (a)
        if((!isLT0 && isRT0 && !isRB0 && isLB0 && isAsym0) || (isLT0 && !isRT0 && isRB0 && !isLB0 && !isAsym0)){
            cEndpoints.push_back(posLeft);
            cEndpoints.push_back(posTop);
            cEndpoints.push_back(posRight);
            cEndpoints.push_back(posBottom);
            LogProcessorInfo("Asym, Left, Top, Right, Bottom");
        }
        // (b)
        else if((!isLT0 && isRT0 && !isRB0 && isLB0 && !isAsym0) || (isLT0 && !isRT0 && isRB0 && !isLB0 && isAsym0)){
            cEndpoints.push_back(posRight);
            cEndpoints.push_back(posTop);
            cEndpoints.push_back(posLeft);
            cEndpoints.push_back(posBottom);
            LogProcessorInfo("Asym, Right, Top, Left, Bottom");
        }
        else;
    }



    // TODO: asym

    return cEndpoints;
}

}  // namespace inviwo
