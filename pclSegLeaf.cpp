#include <pcl/point_types.h>          //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>            //打开关闭pcd文件的类定义的头文件
#include <pcl/io/ply_io.h>            //打开关闭ply文件的类定义的头文件
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>          //最小二乘法平滑处理类定义头文件
#include <pcl/visualization/pcl_visualizer.h> //其实都有就是路径不明确,所以用cmake构建项目更好一点
#include <pcl/features/normal_3d_omp.h> //使用OMP需要添加的头文件

//各种滤波器
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器
#include<pcl/filters/passthrough.h>//直通滤波
//欧式聚类
#include <pcl/segmentation/extract_clusters.h>


//ranrsc算法要用的包
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Dense>
#include <pcl/filters/project_inliers.h>//投影
#include <pcl/ModelCoefficients.h>


// liblas库
#include <liblas/liblas.hpp>
//文件的读取、判断等处理
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <limits.h>

//语句的读入、输出
#include <iostream>
#include <string>
#include <algorithm> //求最大值最小值的库
using namespace std;
//求角运算
#include <cmath>

// 将一个适当类型的输入文件加载到对象PointCloud中，并使其成为全局变量
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//I是强度信息
//去噪后的结果
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//法方向过滤且再次去噪后的结果
pcl::PointCloud<pcl::PointNormal>::Ptr cloudLeaf(new pcl::PointCloud<pcl::PointNormal>);

string fileName=""; //输出结果的文件名

//清空文件夹里的文件
void cleanFile()
{
  char currPath[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", currPath, sizeof(currPath));
    if (len != -1) {
        currPath[len] = '\0';
        vector<std::string> vectorPath;
        stringstream ssPath(currPath);
        string temp = "";
        string currPathName = "";
        while (getline(ssPath,temp, '/')){
          vectorPath.push_back(temp);
        }
        for(int i=0;i<vectorPath.size()-2;i++){
           currPathName+=vectorPath[i]+"/";
        }
        currPathName+="data";
        string order="rm -rf "+currPathName+"/01/*";
        system(order.c_str());
        order="rm -rf "+currPathName+"/02/*";
        system(order.c_str());
        order="rm -rf "+currPathName+"/others/*";
        system(order.c_str());


    } else {
        std::cerr << "执行文件夹的文件结构遭破坏" << std::endl;
    }
  //  cout<<currPath<<endl;
  // string dataFile="../data";
  // string order="rm -rf"+dataFile+"/*";
  // system(order.c_str());

}
//将las文件转为pcd格式
void lasToPcd()
{
  string filePath;//定义文件路径
  cout<<"请输入las格式的点云文件地址:"<<endl;
  cin>>filePath;
  // fileName=filePath;
  // 读取一个 las 点云文件
  //打开las文件
	std::ifstream ifs;
	ifs.open(filePath, std::ios::in | std::ios::binary);
	if (!ifs.is_open()) 
	cout<<"文件无法打开";
	liblas::ReaderFactory f;
	//使用ReaderFactory从stream中构造Reader而不是直接调用Reader构造函数，因为las可能是压缩的
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();    //读取文件头
	int pointsNum = header.GetPointRecordsCount();    //读取点数


    cloud->width = pointsNum;//点的数量（无序点的情况下），这里把las文件中头部点云数量信息直接赋予给它
    cloud->height = 1;   //说明是无序点云
    cloud->is_dense = false; //判断点集是否密集且有效
    cloud->resize(cloud->width * cloud->height);
    int i = 0;
    while (reader.ReadNextPoint()) 
    {
    liblas::Point const& p = reader.GetPoint();
     //四个参数都是浮点数
     cloud->points[i].x = p.GetX();
     cloud->points[i].y = p.GetY();
     cloud->points[i].z = p.GetZ();
    //  cloud->points[i].intensity = p.GetIntensity();
     i++;
    }
    ifs.close();
	//保存为ASICC格式，可以直接用记事本打开
	pcl::io::savePCDFileASCII("../data/others/output.pcd", *cloud);

}

//将txt文件转为pcd格式
void txtToPcd()
{
  string filePath;//定义文件路径
  cout<<"请输入txt格式的点云文件地址:"<<endl;
  cin>>filePath;
  fileName=filePath;
  //打开txt文件
	std::ifstream ifs;
	ifs.open(filePath, ios::in);
	if (!ifs.is_open()) 
	cout<<"文件无法打开";

  else
  {
    string line;
    vector<int> data;
		while (getline(ifs, line))
		{
			std::stringstream ss(line);
			string tempS;
			int n = 0;
			while (getline(ss, tempS, ' ')&&n<3)
			{
        data.push_back(atoi(tempS.c_str()));
        n++;
      }
		}
    int size=data.size()/3;
    cloud->width = size;//点的数量（无序点的情况下），这里把las文件中头部点云数量信息直接赋予给它
    cloud->height = 1;   //说明是无序点云
    cloud->resize(cloud->width * cloud->height);
    for (int i = 0; i <size; ++i)
    {
      cloud->points[i].x = data[i * 3];
      cloud->points[i].y = data[i * 3 + 1];
      cloud->points[i].z = data[i * 3 + 2];
    }
  }
  ifs.close();
	//保存为ASICC格式，可以直接用记事本打开
	pcl::io::savePCDFileASCII("../data/others/output.pcd", *cloud);
}


void createCloud()
{
  int methodID=0;
  // cout<<"[1]las格式数据;"<<endl;
  // cout<<"[2]pcd格式数据;"<<endl;
  // cout<<"[3]txt格式数据;"<<endl;
  // cout<<"请输入文件格式对应的序号:"<<endl;
  // cin>>methodID;

  methodID=3;  //一般为txt格式


  if(methodID==1) 
  {
   lasToPcd();
  }
  else if(methodID==3) 
  {
   txtToPcd();
  }
  else if(methodID==2) 
  {
    string filePath;//定义文件路径
    cout<<"请输入pcd格式的点云文件地址:"<<endl;
    cin>>filePath;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *cloud) == -1) 
    {
      cout<<"读取失败"<<endl;
    }
  }
  
 }

//去噪
void deNoisie()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_A (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_B(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_C (new pcl::PointCloud<pcl::PointXYZ>);
    int meanNumber=100;//定义去噪时取平均值的点数
    // cout<<"请输入去噪时，查询的邻近点数:"<<endl;
    // cin>>meanNumber;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   // 设置输入点云
    sor.setMeanK(meanNumber);           // 设置平均值估计所需的点数
    sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
    sor.filter(*cloud_filtered_A); // 执行滤波操作
    
    //直通滤波，过滤地面点
    pcl::PassThrough<pcl::PointXYZ> passX; // 声明直通滤波
    passX.setInputCloud(cloud_filtered_A); 			// 传入点云数据
    passX.setFilterFieldName("x"); 		// 设置操作的坐标轴
    passX.setFilterLimits(0, 2900); 	// 设置坐标范围，距离雷达0到2.9米的数据需要保留
    passX.setFilterLimitsNegative(false); // 保留范围内的数据
    passX.filter(*cloud_filtered_B);  		// 进行滤波输出
    
    //直通滤波，过滤两边点
    pcl::PassThrough<pcl::PointXYZ> passY; // 声明直通滤波
    passY.setInputCloud(cloud_filtered_B); 			// 传入点云数据
    passY.setFilterFieldName("y"); 		// 设置操作的坐标轴
    passY.setFilterLimits(-2500,1000); 	// 设置坐标范围
    passY.setFilterLimitsNegative(false); // 保留范围内的数据
    passY.filter(*cloud_filtered_C);  		// 进行滤波输出

    //直通滤波，过滤两边点
    pcl::PassThrough<pcl::PointXYZ> passZ; // 声明直通滤波
    passZ.setInputCloud(cloud_filtered_C); 			// 传入点云数据
    passZ.setFilterFieldName("z"); 		// 设置操作的坐标轴
    passZ.setFilterLimits(-700,500); 	// 设置坐标范围
    passZ.setFilterLimitsNegative(false); // 保留范围内的数据
    passZ.filter(*cloud_filtered);  		// 进行滤波输出

    pcl::io::savePCDFile("../data/others/denoisieResult.pcd", *cloud_filtered);
}

//计算法方向
void computeNormal()
{
    //------------------计算法线----------------------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 创建一个新的点云，包含点的坐标和法线信息
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(10);//设置openMP的线程数
    //n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    // n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
    n.setRadiusSearch(48);//半径搜素
    n.compute(*normals);//开始进行法向计
    // 将原始点云和计算出的法向量合并
    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);

    pcl::io::savePCDFile("../data/others/normalResult.pcd", *cloud_with_normals);

    //------------------根据竖直方向上的法方向值进行分割----------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudNoiseLeaf(new pcl::PointCloud<pcl::PointNormal>);
    //直通滤波，过滤掉较为竖直的点
    pcl::PassThrough<pcl::PointNormal> pass; // 声明直通滤波
    pass.setInputCloud(cloud_with_normals); 			// 传入点云数据
    pass.setFilterFieldName("normal_x"); 		// 设置操作的坐标轴
    pass.setFilterLimits(-0.5, 0.5); 	// 设置范围，方向在<-0.5或>0.5的为相对水平的树叶
    pass.setFilterLimitsNegative(true); // 去除范围内的数据
    pass.filter(*cloudNoiseLeaf);  		// 进行滤波输出

    //对叶片场景再去噪
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(cloudNoiseLeaf);   // 设置输入点云
    sor.setMeanK(100);           // 设置平均值估计所需的点数
    sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值
    sor.filter(*cloudLeaf); // 执行滤波操作

    pcl::io::savePCDFile("../data/others/leafResult.pcd", *cloudLeaf);


}

//用欧式聚类进行分割，返回一个文件路径集合
vector<string> euclideanCluster()
{
  double euSearchRadius=10;
  // cout<<"请输入聚类时搜索的半径:"<<endl;
  // cin>>euSearchRadius;
  int minNumber=400;
  // cout<<"请输入一个聚类需要的最少点数:"<<endl;
  // cin>>minNumber;
  int maxNumber=15000;
  // cout<<"请输入一个聚类需要的最多点数:"<<endl;
  // cin>>maxNumber;
  string dataFilepath="../data/01";
  // cout<<"请输入聚类结果的存储路经(文件夹路径必须存在，路径结尾不用加'/'):"<<endl;
  // cin>>dataFilepath;
	// 创建KdTreee对象作为搜索方法

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloudLeaf);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
  ec.setClusterTolerance (euSearchRadius);   // 设置近邻搜索的半径
  ec.setMinClusterSize (minNumber);     //设置一个聚类需要最少的点数目
  ec.setMaxClusterSize (maxNumber);  //设置一个聚类需要最多的点数目
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudLeaf);
  ec.extract (cluster_indices);

  int j = 0;
  vector<string> filePathSet={};
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloudLeaf->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    // std::cout << "当前类有 " << cloud_cluster->points.size () << " 个点" << std::endl;
    std::stringstream ss;
    int k=j+1;
    ss <<dataFilepath<< "/cloud_cluster_" << k<< ".pcd";
    pcl::io::savePCDFile(ss.str (), *cloud_cluster);
    filePathSet.push_back(ss.str ());
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  cout <<"共识别出"<<j<<"个叶片"<<endl;
  return filePathSet;
}

//计算叶片倾角
void computeAngle(vector<string> dataFilepath)
{
	string result = "";
	stringstream ssPath(fileName);
	string temp = "";
  string outFileName = "";
	while (getline(ssPath,temp, '/')){
		/*cout << temp << endl;*/
	}

  for(auto s : temp){
    if(s=='.') break;
    outFileName+=s;
  }
	outFileName+="叶片信息报告";
  string pathName="../data/"+outFileName+".txt";
  ofstream outfile;
	outfile.open(pathName);


  int i=1;
  double maxAngle=0;
  double minAngle=numeric_limits<double>::max();
  double avgAngle=0;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);//暂时忽略强度信息
  for(auto file : dataFilepath){
    if (pcl::io::loadPCDFile<pcl::PointNormal> (file, *cloud) == -1) 
    {
      cout<<"读取失败"<<endl;
    }
    else
    {
      // 创建一个KD树
	    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	    // 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

	    // 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;

	    mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

      //设置参数
      double mlsSearchRadius=30;//定义搜索半径
      mls.setInputCloud(cloud);
      mls.setPolynomialOrder(1);//1次多项式拟合
      mls.setSearchMethod(tree);
      mls.setSearchRadius(mlsSearchRadius);
      mls.process(*cloud_with_normals);
      pcl::io::savePCDFile ("../data/02/"+to_string(i)+"mls.pcd", *cloud_with_normals);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
      pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/02/"+to_string(i)+"mls.pcd", *cloudB);

      //------------------RANRSC计算竖直方向夹角----------------------
      std::vector<int> inliers;
      pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloudB));

      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
      ransac.setDistanceThreshold (6); //6毫米
      ransac.computeModel();
      ransac.getInliers(inliers);

      // 创建一个Eigen::VectorXf对象，用于存储平面模型的系数
      Eigen::VectorXf sacCoefficients;
      // 获取平面模型的系数
      ransac.getModelCoefficients(sacCoefficients);
      // // 打印平面模型的系数
      // std::cout << "平面模型的系数: " << sacCoefficients << std::endl;
  
      //定义模型系数对象，并填充对应的数据
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      coefficients->values.resize(4);
      coefficients->values[0] = sacCoefficients[0];
      coefficients->values[1] = sacCoefficients[1];
      coefficients->values[2] = sacCoefficients[2];
      coefficients->values[3] = sacCoefficients[3];


      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setInputCloud(cloudB);
      proj.setModelCoefficients(coefficients);
      // proj.setModelType(pcl::SACMODEL_PLANE);
      // proj.filter(*cloud_projected);

      double x=coefficients->values[0];
      double y=coefficients->values[1];
      double z=coefficients->values[2];    //法向量的Z值
      
      //归一化
      double nx= x/sqrt(x*x+y*y+z*z);      
      double ny= y/sqrt(x*x+y*y+z*z);   
      double nz= z/sqrt(x*x+y*y+z*z);  
      Eigen::Vector3f  planeNorm(nx,ny,nz);

      //计算夹角
      const double PI=3.1415926;
      double theta=acos(nx) * (180 / PI); //需要将弧度转角度
      theta=theta>=90?180-theta:theta; //限定到[0,90)
      avgAngle+=theta;
      maxAngle=std::max(maxAngle,theta);
      minAngle=std::min(minAngle,theta);
      string tempStr=to_string(theta);

      string angle = tempStr.substr(0,tempStr.find('.')+3) + "度";
      string info = "第" + to_string(i) + "个叶片，倾角：" + angle + ";\n";
      outfile << info;
      i++;
    }
  }
  avgAngle=avgAngle/i;
  string tempMaxAngleStr=to_string(maxAngle );
  string maxAngleStr = tempMaxAngleStr.substr(0,tempMaxAngleStr.find('.')+3) + "度;  ";
  string tempMinAngleStr=to_string(minAngle);
  string minAngleStr = tempMinAngleStr.substr(0,tempMinAngleStr.find('.')+3) + "度;  ";
  string tempAvgAngleStr=to_string(avgAngle );
  string avgAngleStr = tempAvgAngleStr.substr(0,tempAvgAngleStr.find('.')+3) + "度;";

  outfile << "\n"<<"叶片数量："<<i-1<<"个;"<<"\n"<<"其中，最大倾角："<<maxAngleStr<<"最小倾角："<<minAngleStr<<"平均倾角："<<avgAngleStr;
  outfile.close();
}


int main(int argc, char** argv)
{
  cleanFile();
	createCloud();
  deNoisie();
  computeNormal();
  vector<string> filePathSet=euclideanCluster();
  computeAngle(filePathSet);
	return 0;
}