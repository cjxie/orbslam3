echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3

cd ../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3

cd ../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3

cd ../../

echo "Uncompress vocabulary ..."

cd Vocabulary
if [ -f "*.txt" ]; then
	tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3
