#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include "config.h"
#include "core.h"


using namespace config;


int main(int /*argc*/, char **/*argv*/)
{
    // Repeat the benchmark N times:
    const size_t benchRepetitions = 10;
    // Depending on the number of bodies, simulation M time steps:
    // static const std::vector<std::pair<size_t, size_t>> countsIterations = {
    //     { 100, 50 },
    //     { 500, 50 },
    //     { 1000, 50 },
    //     { 5000, 50 },
    //     { 10000, 50 },
    //     { 50000, 50 },
    //     { 100000, 50 },
    //     { 500000, 20 },
    //     { 1000000, 20 },
    //     { 5000000, 20 },
    // };
    static const auto countsIterations = [] () {
        std::vector<std::pair<size_t, size_t>> its;
        for (size_t i = 5000; i <= 100000; i += 5000) {
            its.emplace_back(i, 50);
        }
        return its;
    }();
    static const std::string dirName = "source_points";
    static const std::string bodyScheme = "sphere";
    auto bodyFileName = [] (size_t bodyCount) {
        return dirName + "/" + bodyScheme + "_" + std::to_string(bodyCount) + "_bodies";
    };
    boost::filesystem::create_directory(dirName);
    for (const auto & pair : countsIterations) {
        const size_t count = pair.first;
        const auto fileName = bodyFileName(count);
        if (boost::filesystem::exists(fileName)) {
            continue;
        }
        Model model;
        model.visualMode = false;
        model.outputFileName = fileName;
        if (!model.init(bodyScheme, count)) {
            std::cerr << "Model setup failed" << std::endl;
            return 1;
        }
        if (!model.exportBodies()) {
            std::cerr << "Body export failed. Target file: " << model.outputFileName << std::endl;
            return 2;
        }
    }

    struct Measurement {
        size_t numberOfBodies;
        size_t numberOfSimulationSteps;
        std::vector<double> timePerRunSecs;
    };
    std::vector<Measurement> measurements;
    measurements.reserve(countsIterations.size());

    for (const auto & pair : countsIterations) {
        Measurement mes;
        mes.numberOfBodies = pair.first;
        mes.numberOfSimulationSteps = pair.second;
        mes.timePerRunSecs.reserve(benchRepetitions);
        std::cerr << "Number of bodies: " << mes.numberOfBodies << " " << std::flush;
        const auto fileName = bodyFileName(mes.numberOfBodies);
        for (size_t i = 0; i < benchRepetitions; ++i) {
            Model model;
            model.visualMode = false;
            model.setFrameLimit(mes.numberOfSimulationSteps);
            model.inputFileName = fileName;
            if (!model.init("", 0)) {   // read bodies from file
                std::fprintf(stderr,
                    "Model init failed for count %zu at repetition %zu, reading from %s\n",
                    mes.numberOfBodies, i, fileName.c_str());
                return 3;
            }
            model.benchMode();
            mes.timePerRunSecs.push_back(model.totalRuntimeSeconds());
            std::cerr << "." << std::flush;
        }
        std::cerr << std::endl;
        measurements.emplace_back(std::move(mes));
    }

    for (size_t i = 0; i < measurements.size(); ++i) {
        std::cout << measurements[i].numberOfBodies;
        if (i < measurements.size() - 1) {
            std::cout << ";";
        }
    }
    std::cout << std::endl;

    for (size_t i = 0; i < measurements.size(); ++i) {
        std::cout << measurements[i].numberOfSimulationSteps;
        if (i < measurements.size() - 1) {
            std::cout << ";";
        }
    }
    std::cout << std::endl;

    for (size_t r = 0; r < benchRepetitions; ++r) {
        for (size_t i = 0; i < measurements.size(); ++i) {
            std::cout << measurements[i].timePerRunSecs[r];
            if (i < measurements.size() - 1) {
                std::cout << ";";
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
