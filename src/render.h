#pragma once

#include <unordered_map>
#include <io2d.h>
#include "route_model.h"

using namespace std::experimental;

/**
 * The Render class is responsible for rendering the map and route using the io2d library.
 */
class Render {
public:
    /**
     * Constructor: Initializes the Render class with a RouteModel.
     * @param model The RouteModel containing map and route data.
     */
    Render(RouteModel& model);

    /**
     * Displays the map and route on the given output surface.
     * @param surface The io2d output surface to render on.
     */
    void Display(io2d::output_surface& surface);

private:
    // Builds representations for different road types.
    void BuildRoadReps();

    // Builds brushes for different land use types.
    void BuildLanduseBrushes();

    // Draws buildings on the output surface.
    void DrawBuildings(io2d::output_surface& surface) const;

    // Draws highways on the output surface.
    void DrawHighways(io2d::output_surface& surface) const;

    // Draws railways on the output surface.
    void DrawRailways(io2d::output_surface& surface) const;

    // Draws leisure areas on the output surface.
    void DrawLeisure(io2d::output_surface& surface) const;

    // Draws water bodies on the output surface.
    void DrawWater(io2d::output_surface& surface) const;

    // Draws land use areas on the output surface.
    void DrawLanduses(io2d::output_surface& surface) const;

    // Draws the start position on the output surface.
    void DrawStartPosition(io2d::output_surface& surface) const;

    // Draws the end position on the output surface.
    void DrawEndPosition(io2d::output_surface& surface) const;

    // Draws the calculated path on the output surface.
    void DrawPath(io2d::output_surface& surface) const;

    // Converts a way (road or path) into an io2d path.
    io2d::interpreted_path PathFromWay(const Model::Way& way) const;

    // Converts a multipolygon (e.g., building or water body) into an io2d path.
    io2d::interpreted_path PathFromMP(const Model::Multipolygon& mp) const;

    // Creates a line path for rendering.
    io2d::interpreted_path PathLine() const;

    // Reference to the RouteModel containing map and route data.
    RouteModel& m_Model;

    // Scale factor for rendering.
    float m_Scale = 1.f;

    // Conversion factor from meters to pixels.
    float m_PixelsInMeter = 1.f;

    // Transformation matrix for scaling and translating the map.
    io2d::matrix_2d m_Matrix;

    // Brushes and stroke properties for rendering map features.
    io2d::brush m_BackgroundFillBrush{ io2d::rgba_color{238, 235, 227} };  // Background color

    io2d::brush m_BuildingFillBrush{ io2d::rgba_color{208, 197, 190} };    // Building fill color
    io2d::brush m_BuildingOutlineBrush{ io2d::rgba_color{181, 167, 154} }; // Building outline color
    io2d::stroke_props m_BuildingOutlineStrokeProps{ 1.f };                // Building outline stroke width

    io2d::brush m_LeisureFillBrush{ io2d::rgba_color{189, 252, 193} };     // Leisure area fill color
    io2d::brush m_LeisureOutlineBrush{ io2d::rgba_color{160, 248, 162} };  // Leisure area outline color
    io2d::stroke_props m_LeisureOutlineStrokeProps{ 1.f };                 // Leisure area outline stroke width

    io2d::brush m_WaterFillBrush{ io2d::rgba_color{155, 201, 215} };       // Water body fill color

    io2d::brush m_RailwayStrokeBrush{ io2d::rgba_color{93, 93, 93} };      // Railway stroke color
    io2d::brush m_RailwayDashBrush{ io2d::rgba_color::white };             // Railway dash color
    io2d::dashes m_RailwayDashes{ 0.f, {3.f, 3.f} };                       // Railway dash pattern
    float m_RailwayOuterWidth = 3.f;                                       // Railway outer stroke width
    float m_RailwayInnerWidth = 2.f;                                       // Railway inner stroke width

    // Represents a road with its brush, dash pattern, and width.
    struct RoadRep {
        io2d::brush brush{ io2d::rgba_color::black };  // Road color
        io2d::dashes dashes{};                         // Road dash pattern
        float metric_width = 1.f;                      // Road width in meters
    };

    // Map of road types to their representations.
    std::unordered_map<Model::Road::Type, RoadRep> m_RoadReps;

    // Map of land use types to their brushes.
    std::unordered_map<Model::Landuse::Type, io2d::brush> m_LanduseBrushes;
};