extern "C" {
    bool init(const char* urdf);
    void solve(double* robot_state, double* drone_current, double* drone_goal, const char* transform_name, const double* goal_x, const double* goal_q, const double* velocity, bool* errors, const double* plane_normals, const double* plane_points, const uint8_t* plane_number);
}