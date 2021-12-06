extern "C" {
    bool init(const char* urdf);
    void solve(double* q, const char* transform_name, const double* goal_x, const double* goal_q);
}