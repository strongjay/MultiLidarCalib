
python3 -m venv venv && source venv/bin/activate 
source ./venv/bin/activate
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple # https://pypi.mirrors.ustc.edu.cn/simple/
pip install -e .

cd ./TEASER-plusplus && mkdir -p build && cd build && cmake -DTEASERPP_PYTHON_VERSION=3.10 ..
# 源码 ./TEASER-plusplus/test/CMakeLists.txt 中，可以将 add_subdirectory(teaser) 注释掉，否则会报错
make teaserpp_python
cd python/
pip install .
