
#include <iostream>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

const float fx = 2892.843;
const float fy = 2882.249;
const int w = 1600;
const int h = 1200;
const float cx = 824.4251;
const float cy = 605.18715;

void dtu_to_cameras_txt(const std:: string & out_folder) {

  std::ofstream cameras_txt (out_folder+"/cameras.txt");
  //intrinsics calibration fixed

  cameras_txt << "0 PINHOLE " << w << " " << h << " " << fx << " " << fy << " " << cx << " " << cy<<std::endl;
  cameras_txt.close();
} 

void dtu_to_images_txt(const std:: string & pathImages,const std:: string & pathCalib, const std:: string & out_folder) {


  std::ofstream images_txt (out_folder+"/images.txt");

  //Path Prefixes/postfixes
  const std::string image_prefix = "rect_";
  const std::string image_postfix = "_3_r5000.png";
  const std::string pose_prefix = "pos_";
  const std::string pose_postfix = ".txt";
  const int n_zeros = 3;

  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  K(0,0) = fx;
  K(1,1) = fy;
  K(0,2) = cx;
  K(1,2) = cy;
  K(2,2) = 1.0;

  Eigen::Matrix3f K_inv = K.inverse(); //TODO check

  for (int i = 1; i <= 64; ++i){
    std::string number = std::string(n_zeros - std::to_string(i).length(), '0') + std::to_string(i);

    std::ifstream camFile(pathCalib  + "/" + pose_prefix  + number + pose_postfix);
    std::ifstream imgFile(pathImages + "/" + image_prefix + number + image_postfix);
    
    Eigen::Matrix<float,3,4> cameraMatrix;
    if(camFile.is_open() && imgFile.is_open()){
    //Reading the transpose rotation  since the file stores the inverse of the Hartley Zisserman convention
      camFile >> cameraMatrix(0,0) >> cameraMatrix(0,1) >> cameraMatrix(0,2) >> cameraMatrix(0,3);
      camFile >> cameraMatrix(1,0) >> cameraMatrix(1,1) >> cameraMatrix(1,2) >> cameraMatrix(1,3);
      camFile >> cameraMatrix(2,0) >> cameraMatrix(2,1) >> cameraMatrix(2,2) >> cameraMatrix(2,3);

      Eigen::Matrix<float,3,4> extrinsics;
      extrinsics = K_inv * cameraMatrix;
      
      Eigen::Matrix3f rotation;
      Eigen::Vector3f traslation;
      for (int curR = 0; curR < 3; ++curR) {
        for (int curC = 0; curC < 3; ++curC) {
          rotation(curR,curC) = extrinsics(curR,curC);
        }
      }
      for (int curR = 0; curR < 3; ++curR) {
        traslation(curR) = extrinsics(curR,3);
      }

      Eigen::Quaternionf q(rotation);


      images_txt << i << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z ()
          << " " << traslation(0) << " " << traslation(1) << " " << traslation(2)
          << " 0 " << image_prefix + number + image_postfix << std::endl;
      images_txt << std::endl;
    }
  }

  images_txt.close();
}


void dtu_to_points_txt(const std:: string & out_folder){

  std::ofstream points_txt (out_folder+"/points3D.txt");
  points_txt.close();
}


int main(int argc, char const *argv[])
{
  
  if(argc==4){
    dtu_to_cameras_txt(std::string(argv[3]));
    dtu_to_images_txt(std::string(argv[1]), std::string(argv[2]), std::string(argv[3]));
    dtu_to_points_txt(std::string(argv[3]));

  }else{
    std::cout<< "Wrong Arguments" << std::endl;
    std::cout<< "Usage:" << std::endl;
    std::cout<< "dtu_to_colmap pathImages pathcalib outputfolder" << std::endl;
  }


  return 0;
}
