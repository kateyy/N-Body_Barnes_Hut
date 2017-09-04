#!/bin/bash

set -e

repetitions=3
numBodies=5000000
simulationSteps=13


git checkout baseline

# Slow, "convenient" baseline
echo "baseline: CoM=Body, dynamic allocs"
cmake -DOPTION_ALLOCATOR=std \
    -DOPTION_CENTER_OF_MASS_IS_BODY=ON -DOPTION_LESS_DYNAMIC_ALLOCS=OFF \
    -DOPTION_PERSISTENT_NODE_VECTOR=OFF .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done

# optimized baseline
echo "baseline: CoM=Vec3d+double, no dynamic allocs"
cmake -DOPTION_ALLOCATOR=std \
    -DOPTION_CENTER_OF_MASS_IS_BODY=OFF -DOPTION_LESS_DYNAMIC_ALLOCS=ON \
    -DOPTION_PERSISTENT_NODE_VECTOR=ON .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done


# Slow, "convenient" tcmalloc
echo "tcmalloc: CoM=Body, dynamic allocs"
cmake -DOPTION_ALLOCATOR=tcmalloc \
    -DOPTION_CENTER_OF_MASS_IS_BODY=ON -DOPTION_LESS_DYNAMIC_ALLOCS=OFF \
    -DOPTION_PERSISTENT_NODE_VECTOR=OFF .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done

# optimized tcmalloc
echo "tcmalloc: CoM=Vec3d+double, no dynamic allocs"
cmake -DOPTION_ALLOCATOR=tcmalloc \
    -DOPTION_CENTER_OF_MASS_IS_BODY=OFF -DOPTION_LESS_DYNAMIC_ALLOCS=ON \
    -DOPTION_PERSISTENT_NODE_VECTOR=ON .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done


git checkout PGASUS

# Slow, "convenient" PGASUS
echo "PGASUS: CoM=Body, dynamic allocs"
cmake -DOPTION_PGASUS_STATIC=ON -DOPTION_PGASUS_WITH_POLICIES=ON \
    -DOPTION_CENTER_OF_MASS_IS_BODY=ON -DOPTION_LESS_DYNAMIC_ALLOCS=OFF \
    -DOPTION_PERSISTENT_NODE_VECTOR=OFF .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done

# optimized PGASUS
echo "PGASUS: CoM=Vec3d+double, no dynamic allocs"
cmake -DOPTION_PGASUS_STATIC=ON -DOPTION_PGASUS_WITH_POLICIES=ON \
    -DOPTION_CENTER_OF_MASS_IS_BODY=OFF -DOPTION_LESS_DYNAMIC_ALLOCS=ON \
    -DOPTION_PERSISTENT_NODE_VECTOR=ON .

ninja

for i in `seq 1 $repetitions`;
do
    OMP_PLACES=sockets ./nbody -f source_points/sphere_5000000_bodies \
        -n $numBodies -i $simulationSteps
done
