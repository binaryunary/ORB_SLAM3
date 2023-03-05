echo "Configuring and building Thirdparty/DBoW2 ..."
pushd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j $(nproc)
popd

echo "Configuring and building Thirdparty/g2o ..."
pushd Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j $(nproc)
popd

echo "Configuring and building Thirdparty/Sophus ..."
pushd Thirdparty/Sophus
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j $(nproc)


