#include <chrono>
#include <iostream>
#include <string>

namespace ORB_SLAM2
{
    class TicToc
    {
    public:
        TicToc()
        {
            tic();
        }


        void tic()
        {
            mtpStart = std::chrono::steady_clock::now();
        }


        double toc(const std::string &log="")
        {
            mtpEnd = std::chrono::steady_clock::now();
            double tt = std::chrono::duration_cast<std::chrono::duration<double> >(mtpEnd-mtpStart).count();
            std::cout << log << ", time cost: " << tt << " s" << std::endl;
            mtpEnd = mtpStart;
            return tt;
        }

    private:
        std::chrono::steady_clock::time_point mtpStart;
        std::chrono::steady_clock::time_point mtpEnd;
    };
}