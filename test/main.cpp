#include <iostream>
#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <vector>

#include "../adaptive_mesh.h"

void load_wavefront_obj(
    std::vector<unsigned int> &tri2vtx,
    std::vector<double> &vtx2xyz,
    const std::filesystem::path &file_path) {
  std::ifstream fin;
  fin.open(file_path);
  if (fin.fail()) {
    std::cout << "File Read Fail" << std::endl;
    return;
  }
  vtx2xyz.clear();
  tri2vtx.clear();
  vtx2xyz.reserve(256 * 16);
  tri2vtx.reserve(256 * 16);
  const int BUFF_SIZE = 256;
  char buff[BUFF_SIZE];
  while (fin.getline(buff, BUFF_SIZE)) {
    if (buff[0] == '#') { continue; }
    if (buff[0] == 'v' && buff[1] == ' ') {
      char str[256];
      double x, y, z;
      {
        std::istringstream is(buff);
        is >> str >> x >> y >> z;
//        sscanf(buff, "%s %lf %lf %lf", str, &x, &y, &z);
      }
      vtx2xyz.push_back(x);
      vtx2xyz.push_back(y);
      vtx2xyz.push_back(z);
    }
    if (buff[0] == 'f') {
      char str[256];
      int i0, i1, i2;
      {
        std::istringstream is(buff);
        is >> str >> i0 >> i1 >> i2;
//       sscanf(buff, "%s %d %d %d", str, &i0, &i1, &i2);
      }
      tri2vtx.push_back(i0 - 1);
      tri2vtx.push_back(i1 - 1);
      tri2vtx.push_back(i2 - 1);
    }
  }
}

void Write_Obj_UniformMesh(
    const std::string &file_path,
    const std::vector<unsigned int>& elem_vtx,
    const  Eigen::Matrix<double, -1, 3, Eigen::RowMajor>& vtx_xyz ) {
  std::ofstream fout(
      file_path.c_str(),
      std::ofstream::out);
  for (Eigen::Index ip = 0; ip < vtx_xyz.rows(); ip++) {
    fout << "v ";
    fout << vtx_xyz(ip,0) << " ";
    fout << vtx_xyz(ip,1) << " ";
    fout << vtx_xyz(ip,2) << std::endl;
  }
  for (unsigned int iel = 0; iel < elem_vtx.size() / 3; iel++) {
    fout << "f ";
    for (unsigned int ino = 0; ino < 3; ++ino ) {
      fout << elem_vtx[iel*3+ino] + 1 << " ";
    }
    fout << std::endl;
  }
}

int main() {
  std::cout << "hoge" << std::endl;
  std::vector<unsigned int> a_tri2vtx;
  Eigen::Matrix<double, -1, 3, Eigen::RowMajor> a_vtx2xyz;
  {
    std::vector<double> vtx2xyz_stl;
    load_wavefront_obj(a_tri2vtx, vtx2xyz_stl,
                       std::string(PATH_PROJECT_SOURCE_DIR) + "/bunny_1k.obj");
    std::cout << vtx2xyz_stl.size() << std::endl;
    std::cout << a_tri2vtx.size() << std::endl;
    a_vtx2xyz = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        vtx2xyz_stl.data(),
        static_cast<unsigned int>(vtx2xyz_stl.size() / 3), 3);
  }

  adaptive::Mesh am(a_tri2vtx, a_vtx2xyz);
  {
    am.split_edge({286, 22,
                   369, 98,
                   357, 135,
                   95, 391,
                   0, 381});
  }
  const std::vector<unsigned int> b_tri2vtx = am.F();
  Eigen::Matrix<double, -1, 3, Eigen::RowMajor> b_vtx2xyz = am.V();
  std::cout << b_tri2vtx.size() / 3 << std::endl;
  Write_Obj_UniformMesh("hoge.obj", b_tri2vtx, b_vtx2xyz);
}