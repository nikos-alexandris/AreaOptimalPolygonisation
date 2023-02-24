# gcc 9.4.0
# cgal 5.5
# boost 1.71

FROM ubuntu:20.04
RUN apt-get update && apt-get install -y \
    gcc g++ gdb make cmake valgrind wget python python-dev libmpfr-dev                          &&\
    cd                                                                                          &&\
    wget https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.gz   &&\
    tar -xvzf boost_1_71_0.tar.gz                                                               &&\
    rm boost_1_71_0.tar.gz                                                                      &&\
    cd boost_1_71_0/                                                                            &&\
    ./bootstrap.sh                                                                              &&\
    ./b2 -j $(nproc) install                                                                    &&\
    cd                                                                                          &&\
    wget https://github.com/CGAL/cgal/archive/refs/tags/v5.5.tar.gz                             &&\
    tar -xvzf v5.5.tar.gz                                                                       &&\
    rm v5.5.tar.gz                                                                              &&\
    cd cgal-5.5/                                                                                &&\
    mkdir build                                                                                 &&\
    cd build                                                                                    &&\
    cmake ..                                                                                    &&\
    make -j $(nproc) install                                                                    &&\
    cd                                                                                          &&\
    echo "export PATH=$PATH:/root/cgal-5.5/Scripts/scripts/" >> /root/.bashrc
COPY . /root/AreaOptimalPolygonisation
CMD ["bash"]