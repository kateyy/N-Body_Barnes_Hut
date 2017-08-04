#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include <PGASUS/malloc.hpp>
#include <PGASUS/base/node.hpp>

#include "config.h"
#include "core.h"


using namespace config;


int main(int argc, char **argv)
{
    const auto & numaNodes = numa::NodeList::logicalNodes();
    // TODO use the largest/fasted node instead?
    auto homeNodeIt = std::find_if(numaNodes.begin(), numaNodes.end(), [] (const numa::Node & node) {
        return node.memorySize() > 0; });
    if (homeNodeIt == numaNodes.end()) {
        std::cerr << "No valid NUMA node found." << std::endl;
        return 1;
    }
    const numa::Node homeNode = *homeNodeIt;
    // By default all memory is allocated on a "home" node.
    const numa::PlaceGuard numaGuard{ homeNode };

    enum class Mode
    {
        timePerSize,
        timePerStep
    };
    size_t defaultNumBodies = 50000;
    std::string defaultScheme = "sphere";
    size_t defaultFrameLimit = 50;
    const Mode benchMode = [argc, argv, &defaultNumBodies, &defaultScheme, &defaultFrameLimit] () {
        if (argc == 1) {
            return Mode::timePerSize;
        }
        if (argc >= 2) {
            const auto modeStr = std::string(argv[1]);
            if (modeStr == "timePerSize") { return Mode::timePerSize; }
            if (modeStr == "timePerStep") {
                if (argc >= 3) { defaultNumBodies = std::stoull(argv[2]); }
                if (argc >= 4) { defaultScheme = argv[3]; }
                if (argc >= 5) { defaultFrameLimit = std::stoull(argv[4]); }
                return Mode::timePerStep;
            }
        }
        std::cerr << "Invalid parameters!" << std::endl;
        exit(1);
    }();

    static const std::string dirName = "source_points";
    auto bodyFileName = [] (size_t bodyCount, const std::string &bodyScheme) {
        return dirName + "/" + bodyScheme + "_" + std::to_string(bodyCount) + "_bodies";
    };
    boost::filesystem::create_directory(dirName);

    auto createBodiesFile = [bodyFileName](size_t bodyCount, const std::string &bodyScheme) {
        const auto fileName = bodyFileName(bodyCount, bodyScheme);
        if (boost::filesystem::exists(fileName)) {
            return fileName;
        }
        Model model;
        model.visualMode = false;
        model.outputFileName = fileName;
        if (!model.init(bodyScheme, bodyCount)) {
            std::cerr << "Model setup failed" << std::endl;
            exit(2);
        }
        if (!model.exportBodies()) {
            std::cerr << "Body export failed. Target file: " << model.outputFileName << std::endl;
            exit(3);
        }
        return fileName;
    };


    if (benchMode == Mode::timePerStep) {
        Model model;
        model.visualMode = false;
        model.setFrameLimit(defaultFrameLimit);
        model.inputFileName = createBodiesFile(defaultNumBodies, defaultScheme);
        if (!model.init("", 0)) {   // read bodies from file
            std::cerr << "Model init failed, reading from" << model.inputFileName << std::endl;
            return 3;
        }
        while (true) {
            const auto start = std::chrono::high_resolution_clock::now();
            if (!model.updateUnlocked()) {
                break;
            }
            const auto end = std::chrono::high_resolution_clock::now();
            std::cout << model.frameCount() << ";"
                << timeDiffNanoSecs(start, end) * 1e-6 << std::endl;
        }
        std::cerr << "Total runtime: " << model.totalRuntimeSeconds() << "s" << std::endl;
        return 0;
    }



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
    for (const auto & pair : countsIterations) {
        createBodiesFile(pair.first, defaultScheme);
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
        const auto fileName = bodyFileName(mes.numberOfBodies, defaultScheme);
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
