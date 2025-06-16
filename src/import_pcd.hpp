
pcl::PointCloud<pcl::PointXYZ>::Ptr import_pcd(const std::string& pcd_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "Failed to load pcd file: " << pcd_path << std::endl;
        return nullptr;
    }
    return cloud;
}