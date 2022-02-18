#include <stdint.h>
extern "C" {
    bool init(const char* urdf);
    double solve(double* robot_state, double* drone_current, double* drone_goal, const char* transform_name, const double* goal_x, const double* goal_q, const double* velocity, bool* errors, const double* plane_normals, const double* plane_points, const double* plane_centers, const double* plane_orientations, const double* plane_half_axes, const uint8_t* plane_number, const double* uncertainty);
    void solveDroneOnly(double* robot_state, double* drone_current, double* drone_goal, double* last_drone_goal, bool* errors, const double* plane_normals, const double* plane_points, const double* plane_centers, const double* plane_orientations, const double* plane_half_axes, const uint8_t* plane_number, const double* uncertainty);
    double solveGlobalView(double* robot_state, double* drone_current, bool* errors, const double* plane_normals, const double* plane_points, const double* plane_centers, const double* plane_orientations, const double* plane_half_axes, const uint8_t* plane_number, const double* uncertainty);
    void getMap(double* robot_state, const double* plane_normals, const double* plane_points, const double* plane_centers, const double* plane_orientations, const double* plane_half_axes, const uint8_t* plane_number, const double* uncertainty, double* map);
}