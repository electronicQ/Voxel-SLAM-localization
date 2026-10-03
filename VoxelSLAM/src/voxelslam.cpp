#include "voxelslam.hpp"
#include "global_correction.hpp"
#include "btc_io.hpp"
#include "localization_state.hpp"
#include "mapping_capture.hpp"
#include "prior_map_tracker.hpp"
#include "relocalizer.hpp"
#include "session_aligner.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <new>
#include <stdexcept>
#include <unordered_set>

using namespace std;

namespace {

CorrectionState toCorrectionState(const IMUST &state) {
  CorrectionState result;
  result.R = state.R;
  result.p = state.p;
  result.v = state.v;
  result.bg = state.bg;
  result.ba = state.ba;
  result.g = state.g;
  result.cov = state.cov;
  return result;
}

void updateFromCorrectionState(const CorrectionState &source, IMUST &target) {
  target.R = source.R;
  target.p = source.p;
  target.v = source.v;
  target.bg = source.bg;
  target.ba = source.ba;
  target.g = source.g;
  target.cov = source.cov;
}

}  // namespace

class ResultOutput
{
public:
  static ResultOutput &instance()
  {
    static ResultOutput inst;
    return inst;
  }

  void pub_odom_func(IMUST &xc)
  {
    Eigen::Quaterniond q_this(xc.R);
    Eigen::Vector3d t_this = xc.p;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_this.x(), t_this.y(), t_this.z()));
    q.setW(q_this.w());
    q.setX(q_this.x());
    q.setY(q_this.y());
    q.setZ(q_this.z());
    transform.setRotation(q);
    ros::Time ct = ros::Time::now();
    br.sendTransform(tf::StampedTransform(transform, ct, "/camera_init", "/aft_mapped"));
  }

  void pub_localtraj(PLV(3) &pwld, double jour, IMUST &x_curr, int cur_session, pcl::PointCloud<PointType> &pcl_path)
  {
    pub_odom_func(x_curr);
    pcl::PointCloud<PointType> pcl_send;
    pcl_send.reserve(pwld.size());
    for(Eigen::Vector3d &pw: pwld)
    {
      Eigen::Vector3d pvec = pw;
      PointType ap;
      ap.x = pvec.x();
      ap.y = pvec.y();
      ap.z = pvec.z();
      pcl_send.push_back(ap);
    }
    pub_pl_func(pcl_send, pub_scan);
    
    Eigen::Vector3d pcurr = x_curr.p;

    PointType ap;
    ap.x = pcurr[0];
    ap.y = pcurr[1];
    ap.z = pcurr[2];
    ap.curvature = jour;
    ap.intensity = cur_session;
    pcl_path.push_back(ap);
    pub_pl_func(pcl_path, pub_curr_path);
  }

  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr> &pvec_buf, vector<IMUST> &x_buf, pcl::PointCloud<PointType> &pcl_path, int win_base, int win_count)
  {
    pcl::PointCloud<PointType> pcl_send;
    for(int i=0; i<mgsize; i++)
    {
      for(int j=0; j<pvec_buf[i]->size(); j+=3)
      {
        pointVar &pv = pvec_buf[i]->at(j);
        Eigen::Vector3d pvec = x_buf[i].R*pv.pnt + x_buf[i].p;
        PointType ap;
        ap.x = pvec[0];
        ap.y = pvec[1];
        ap.z = pvec[2];
        ap.intensity = cur_session;
        pcl_send.push_back(ap);
      }
    }

    for(int i=0; i<win_count; i++)
    {
      Eigen::Vector3d pcurr = x_buf[i].p;
      pcl_path[i+win_base].x = pcurr[0];
      pcl_path[i+win_base].y = pcurr[1];
      pcl_path[i+win_base].z = pcurr[2];
    }

    pub_pl_func(pcl_path, pub_curr_path);
    pub_pl_func(pcl_send, pub_cmap);
  }

  void pub_global_path(vector<vector<ScanPose*>*> &relc_bl_buf, ros::Publisher &pub_relc, vector<int> &ids)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pcl::PointXYZI pp;
    int idsize = ids.size();

    for(int i=0; i<idsize; i++)
    {
      pp.intensity = ids[i];
      for(ScanPose* bl: *(relc_bl_buf[ids[i]]))
      {
        pp.x = bl->x.p[0]; pp.y = bl->x.p[1]; pp.z = bl->x.p[2];
        pl.push_back(pp);
      }
    }
    pub_pl_func(pl, pub_relc);
  }

  void pub_globalmap(vector<vector<Keyframe*>*> &relc_submaps, vector<int> &ids, ros::Publisher &pub)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pub_pl_func(pl, pub);
    pcl::PointXYZI pp;

    uint interval_size = 5e6;
    uint psize = 0;
    for(int id: ids)
    {
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
        psize += smps[i]->plptr->size();
    }
    int jump = psize / (10 * interval_size) + 1;

    for(int id: ids)
    {
      pp.intensity = id;
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
      {
        IMUST xx = smps[i]->x0;
        for(int j=0; j<smps[i]->plptr->size(); j+=jump)
        // for(int j=0; j<smps[i]->plptr->size(); j+=1)
        {
          PointType &ap = smps[i]->plptr->points[j];
          Eigen::Vector3d vv(ap.x, ap.y, ap.z);
          vv = xx.R * vv + xx.p;
          pp.x = vv[0]; pp.y = vv[1]; pp.z = vv[2];
          pl.push_back(pp);
        }

        if(pl.size() > interval_size)
        {
          pub_pl_func(pl, pub);
          sleep(0.05);
          pl.clear();
        }
      }
    }
    pub_pl_func(pl, pub);
  }

};

class FileReaderWriter
{
public:
  static FileReaderWriter &instance()
  {
    static FileReaderWriter inst;
    return inst;
  }

  void save_pcd(PVecPtr pptr, IMUST &xx, int count, const string &savename)
  {
    pcl::PointCloud<pcl::PointXYZI> pl_save;
    for(pointVar &pw: *pptr)
    {
      pcl::PointXYZI ap;
      ap.x = pw.pnt[0]; ap.y = pw.pnt[1]; ap.z = pw.pnt[2];
      pl_save.push_back(ap);
    }
    string pcdname = savename + "/" + to_string(count) + ".pcd";
    pcl::io::savePCDFileBinary(pcdname, pl_save); 
  }

  void save_pose(vector<ScanPose*> &bbuf, string &fname, string posename, string &savepath)
  {
    if(bbuf.size() < 100) return;
    int topsize = bbuf.size();

    ofstream posfile(savepath + fname + posename);
    for(int i=0; i<topsize; i++)
    {
      IMUST &xx = bbuf[i]->x;
      Eigen::Quaterniond qq(xx.R);
      posfile << fixed << setprecision(6) << xx.t << " ";
      posfile << setprecision(7) << xx.p[0] << " " << xx.p[1] << " " << xx.p[2] << " ";
      posfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();
      posfile << " " << xx.v[0] << " " << xx.v[1] << " " << xx.v[2];
      posfile << " " << xx.bg[0] << " " << xx.bg[1] << " " << xx.bg[2];
      posfile << " " << xx.ba[0] << " " << xx.ba[1] << " " << xx.ba[2];
      posfile << " " << xx.g[0] << " " << xx.g[1] << " " << xx.g[2];
      for(int j=0; j<6; j++) posfile << " " << bbuf[i]->v6[j];
      posfile << endl;
    }
    posfile.close();

  }

  // The loop clousure edges of multi sessions
  void pgo_edges_io(PGO_Edges &edges, vector<string> &fnames, int io, string &savepath, string &bagname)
  {
    static vector<string> seq_absent;
    Eigen::Matrix<double, 6, 1> v6_init;
    v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    if(io == 0) // read
    {
      ifstream infile(savepath + "edge.txt");
      string lineStr, str;
      vector<string> sts;
      while(getline(infile, lineStr))
      {
        sts.clear();
        stringstream ss(lineStr);
        while(ss >> str)
          sts.push_back(str);
        
        int mp[2] = {-1, -1};
        for(int i=0; i<2; i++)
        for(int j=0; j<fnames.size(); j++)
        if(sts[i] == fnames[j])
        {
          mp[i] = j;
          break;
        }

        if(mp[0] != -1 && mp[1] != -1)
        {
          int id1 = stoi(sts[2]);
          int id2 = stoi(sts[3]);
          Eigen::Vector3d v3; 
          v3 << stod(sts[4]), stod(sts[5]), stod(sts[6]);
          Eigen::Quaterniond qq(stod(sts[10]), stod(sts[7]), stod(sts[8]), stod(sts[9]));
          Eigen::Matrix3d rot(qq.matrix());
          if(mp[0] <= mp[1])
            edges.push(mp[0], mp[1], id1, id2, rot, v3, v6_init);
          else
          {
            v3 = -rot.transpose() * v3;
            rot = qq.matrix().transpose();
            edges.push(mp[1], mp[0], id2, id1, rot, v3, v6_init);
          }
        }
        else
        {
          if(sts[0] != bagname && sts[1] != bagname)
            seq_absent.push_back(lineStr);
        }

      }
    }
    else // write
    {
      ofstream outfile(savepath + "edge.txt");
      for(string &str: seq_absent)
        outfile << str << endl;

      for(PGO_Edge &edge: edges.edges)
      {
        for(int i=0; i<edge.rots.size(); i++)
        {
          outfile << fnames[edge.m1] << " ";
          outfile << fnames[edge.m2] << " ";
          outfile << edge.ids1[i] << " ";
          outfile << edge.ids2[i] << " ";
          Eigen::Vector3d v(edge.tras[i]);
          outfile << setprecision(7) << v[0] << " " << v[1] << " " << v[2] << " ";
          Eigen::Quaterniond qq(edge.rots[i]);
          outfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w() << endl;
        }
      }
      outfile.close();
    }

  }

  // loading the offline map
  void previous_map_names(ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds)
  {
    string premap;
    n.param<string>("General/previous_map", premap, "");
    premap.erase(remove_if(premap.begin(), premap.end(), ::isspace), premap.end());
    stringstream ss(premap);
    string str;
    while(getline(ss, str, ','))
    {
      stringstream ss2(str);
      vector<string> strs;
      while(getline(ss2, str, ':'))
        strs.push_back(str);
      
      if(strs.size() != 2)
      {
        printf("previous map name wrong\n");
        return;
      }

      if(strs[0][0] != '#')
      {
        fnames.push_back(strs[0]);
        juds.push_back(stod(strs[1]));
      }
    }

  }

  void previous_map_read(vector<STDescManager*> &std_managers, vector<vector<ScanPose*>*> &multimap_scanPoses, vector<vector<Keyframe*>*> &multimap_keyframes, ConfigSetting &config_setting, PGO_Edges &edges, ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds, string &savepath, int win_size)
  {
    int acsize = 10; int mgsize = 5;
    n.param<int>("Loop/acsize", acsize, 10);
    n.param<int>("Loop/mgsize", mgsize, 5);

    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
    {
      string fname = savepath + fnames[fn];
      vector<ScanPose*>* bl_tem = new vector<ScanPose*>();
      vector<Keyframe*>* keyframes_tem = new vector<Keyframe*>();
      STDescManager *std_manager = new STDescManager(config_setting);

      std_managers.push_back(std_manager);
      multimap_scanPoses.push_back(bl_tem);
      multimap_keyframes.push_back(keyframes_tem);
      read_lidarstate(fname+"/alidarState.txt", *bl_tem);

      cout << "Reading " << fname << ": " << bl_tem->size() << " scans." << "\n";
      deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> plbuf;
      deque<IMUST> xxbuf;
      pcl::PointCloud<PointType> pl_lc;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_btc(new pcl::PointCloud<pcl::PointXYZI>());

      for(int i=0; i<bl_tem->size() && n.ok(); i++)
      {
        IMUST &xc = bl_tem->at(i)->x;
        string pcdname = fname + "/" + to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr pl_tem(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcdname, *pl_tem);

        xxbuf.push_back(xc);
        plbuf.push_back(pl_tem);
        
        if(xxbuf.size() < win_size)
          continue;
        
        pl_lc.clear();
        Keyframe *smp = new Keyframe(xc);
        smp->id = i;
        PointType pt;
        for(int j=0; j<win_size; j++)
        {
          Eigen::Vector3d delta_p = xc.R.transpose() * (xxbuf[j].p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xxbuf[j].R;

          for(pcl::PointXYZI pp: plbuf[j]->points)
          {
            Eigen::Vector3d v3(pp.x, pp.y, pp.z);
            v3 = delta_R * v3 + delta_p;
            pt.x = v3[0]; pt.y = v3[1]; pt.z = v3[2];
            pl_lc.push_back(pt);
          }
        }

        down_sampling_voxel(pl_lc, voxel_size/10);
        smp->plptr->reserve(pl_lc.size());
        for(PointType &pp: pl_lc.points)
          smp->plptr->push_back(pp);
        keyframes_tem->push_back(smp);
        
        for(int j=0; j<win_size; j++)
        {
          plbuf.pop_front(); xxbuf.pop_front();
        }
      }
      
      cout << "Generating BTC descriptors..." << "\n";

      int subsize = keyframes_tem->size();
      for(int i=0; i+acsize<subsize && n.ok(); i+=mgsize)
      {
        int up = i + acsize;
        pl_btc->clear();
        IMUST &xc = keyframes_tem->at(up - 1)->x0;
        for(int j=i; j<up; j++)
        {
          IMUST &xj = keyframes_tem->at(j)->x0;
          Eigen::Vector3d delta_p = xc.R.transpose() * (xj.p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xj.R;
          pcl::PointXYZI pp;
          for(PointType ap: keyframes_tem->at(j)->plptr->points)
          {
            Eigen::Vector3d v3(ap.x, ap.y, ap.z);
            v3 = delta_R * v3 + delta_p;
            pp.x = v3[0]; pp.y = v3[1]; pp.z = v3[2];
            pl_btc->push_back(pp);
          }
        }

        vector<STD> stds_vec;
        std_manager->GenerateSTDescs(pl_btc, stds_vec, keyframes_tem->at(up-1)->id);
        std_manager->AddSTDescs(stds_vec);
      }
      std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);

      cout << "Read " << fname << " done." << "\n\n";
    }

    vector<int> ids_all;
    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
      ids_all.push_back(fn);

    // gtsam::Values initial;
    // gtsam::NonlinearFactorGraph graph;
    // vector<int> ids_cnct, stepsizes;
    // Eigen::Matrix<double, 6, 1> v6_init;
    // v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    // gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    // build_graph(initial, graph, ids_all.back(), edges, odom_noise, ids_cnct, stepsizes, 1);

    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // gtsam::ISAM2 isam(parameters);
    // isam.update(graph, initial);

    // for(int i=0; i<5; i++) isam.update();
    // gtsam::Values results = isam.calculateEstimate();
    // int resultsize = results.size();
    // int idsize = ids_cnct.size();
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
    //   {
    //     int ord = j - stepsizes[ii];
    //     multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
    //   }
    // }
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(Keyframe *kf: *multimap_keyframes[tip])
    //     kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
    // }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids_all);
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids_all, pub_pmap);

    printf("All the maps are loaded\n");
  }
  
};

class Initialization
{
public:
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  void align_gravity(vector<IMUST> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;
    Eigen::Vector3d n0 = g0 / g0.norm();
    Eigen::Vector3d n1(0, 0, 1);
    if(n0[2] < 0)
      n1[2] = -1;
    
    Eigen::Vector3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
    Eigen::Matrix3d rot = angaxis.matrix();
    g0 = rot * g0;

    Eigen::Vector3d p0 = xs[0].p;
    for(int i=0; i<xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;
      xs[i].R = rot * xs[i].R;
      xs[i].v = rot * xs[i].v;
      xs[i].g = g0;
    }

  }

  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::Imu::Ptr> &imus, double pcl_beg_time, IMUST &extrin_para)
  {
    xc.bg = xl.bg; xc.ba = xl.ba;
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
    Eigen::Matrix3d R_imu(xc.R);
    vector<IMUST> imu_poses;

    for(auto it_imu=imus.end()-1; it_imu!=imus.begin(); it_imu--)
    {
      sensor_msgs::Imu &head = **(it_imu-1);
      sensor_msgs::Imu &tail = **(it_imu); 
      
      angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x), 
                    0.5*(head.angular_velocity.y + tail.angular_velocity.y), 
                    0.5*(head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x), 
                 0.5*(head.linear_acceleration.y + tail.linear_acceleration.y), 
                 0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= xc.bg;
      acc_avr = acc_avr * imupre_scale_gravity - xc.ba;

      double dt = head.header.stamp.toSec() - tail.header.stamp.toSec();
      Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

      acc_imu = R_imu * acc_avr + xc.g;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;
      R_imu = R_imu * Exp_f;

      double offt = head.header.stamp.toSec() - pcl_beg_time;
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    pointVar pv; pv.var.setIdentity();
    if(point_notime)
    {
      for(PointType &ap: pl.points)
      {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
        pvec.push_back(pv);
      }
      return;
    }
    auto it_pcl = pl.end() - 1;
    // for(auto it_kp=imu_poses.end(); it_kp!=imu_poses.begin(); it_kp--)
    for(auto it_kp=imu_poses.begin(); it_kp!=imu_poses.end(); it_kp++)
    {
      // IMUST &head = *(it_kp - 1);
      IMUST &head = *it_kp;
      R_imu = head.R;
      acc_imu = head.ba;
      vel_imu = head.v;
      pos_imu = head.p;
      angvel_avr = head.bg;

      for(; it_pcl->curvature > head.t; it_pcl--)
      {
        double dt = it_pcl->curvature - head.t;
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

        pv.pnt = P_compensate;
        pvec.push_back(pv);
        if(it_pcl == pl.begin()) break;
      }

    }
  }

  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::Imu::Ptr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree*> &surf_map, unordered_map<VOXEL_LOC, OctoTree*> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow*>> &sws, IMUST &x_curr, deque<IMU_PRE*> &imu_pre_buf, IMUST &extrin_para)
  {
    PLV(3) pwld;
    double last_g_norm = x_buf[0].g.norm();
    int converge_flag = 0;

    double min_eigen_value_orig = min_eigen_value;
    vector<double> eigen_value_array_orig = plane_eigen_value_thre;

    min_eigen_value = 0.02;
    for(double &iter: plane_eigen_value_thre)
      iter = 1.0 / 4;

    double t0 = ros::Time::now().toSec();
    double converge_thre = 0.05;
    int converge_times = 0;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue; eigvalue.setZero();
    for(int iterCnt = 0; iterCnt < 10; iterCnt++)
    {
      if(converge_flag == 1)
      {
        min_eigen_value = min_eigen_value_orig;
        plane_eigen_value_thre = eigen_value_array_orig;
      }

      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();

      for(int i=0; i<win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i==0 ? i : i - 1;
        motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

        if(converge_flag == 1)
        {
          for(pointVar &pv: *pvec_buf[i])
            calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for(pointVar &pv: *pvec_buf[i])
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }

        cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
      }

      // LidarFactor voxhess(win_size);
      voxhess.clear(); voxhess.win_size = win_size;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->recut(win_size, x_buf, sws[0]);
        iter->second->tras_opt(voxhess);
      }

      if(voxhess.plvec_voxels.size() < 10)
        break;
      LI_BA_OptimizerGravity opt_lsv;
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
      Eigen::Matrix3d nnt; nnt.setZero();

      printf("%d: %lf %lf %lf: %lf %lf\n", iterCnt, x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm(), fabs(resis[0] - resis[1]) / resis[0]);

      for(int i=0; i<win_size-1; i++)
        delete imu_pre_buf[i];
      imu_pre_buf.clear();

      for(int i=1; i<win_size; i++)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[i-1].bg, x_buf[i-1].ba));
        imu_pre_buf.back()->push_imu(vec_imus[i]);
      }

      if(fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for(Eigen::Matrix3d &iter: voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < 15 ? true : false;

        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
          break;
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if(is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if(converge_flag == 0)
    {
      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();
    }

    printf("mn: %lf %lf %lf\n", eigvalue[0], eigvalue[1], eigvalue[2]);
    Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y, vec_imus[0][0]->angular_velocity.z);
    Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y, vec_imus[0][0]->linear_acceleration.z);
    acc *= 9.8;

    pl_origs.clear(); vec_imus.clear(); beg_times.clear();
    double t1 = ros::Time::now().toSec();
    printf("init time: %lf\n", t1 - t0);

    // align_gravity(x_buf);
    pcl::PointCloud<PointType> pcl_send; PointType pt;
    for(int i=0; i<win_size; i++)
    for(pointVar &pv: *pvec_buf[i])
    {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0]; pt.y = vv[1]; pt.z = vv[2];
      pcl_send.push_back(pt);
    }
    pub_pl_func(pcl_send, pub_init);

    return converge_flag;
  }

};

class VOXEL_SLAM
{
public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;
  double down_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE*> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow*>> sws;

  vector<ScanPose*> *scanPoses;
  mutex mtx_loop;
  deque<ScanPose*> buf_lba2loop, buf_lba2loop_tem;
  vector<Keyframe*> *keyframes;
  int loop_detect = 0;
  unordered_map<VOXEL_LOC, OctoTree*> map_loop;
  IMUST dx;
  pcl::PointCloud<PointType>::Ptr pl_kdmap;
  pcl::KdTreeFLANN<PointType> kd_keyframes;
  int history_kfsize = 0;
  vector<OctoTree*> octos_release;
  std::atomic<int> reset_flag{0};
  bool has_global_correction_generation = false;
  std::uint64_t global_correction_generation = 0;
  int g_update = 0;
  int thread_num = 5;
  int degrade_bound = 10;

  vector<vector<ScanPose*>*> multimap_scanPoses;
  vector<vector<Keyframe*>*> multimap_keyframes;
  volatile int gba_flag = 0;
  int gba_size = 0;
  vector<int> cnct_map;
  mutex mtx_keyframe;
  PGO_Edges gba_edges1, gba_edges2;
  std::atomic<bool> is_finish{false};

  vector<string> sessionNames;
  string bagname, savepath;
  string operation_mode, map_export_path, localization_map_path;
  SavedMap localization_map;
  PriorMapTracker prior_map_tracker;
  mutex historical_map_mutex;
  std::atomic<bool> prior_map_ready{false};
  std::atomic<bool> localization_valid{false};
  mutex frontend_pipeline_mutex;
  mutex localization_state_mutex;
  LocalizationStateMachine localization_machine;
  FrontendSessionEpoch frontend_session_epoch;
  ros::Publisher pub_localization_state;
  ros::Publisher pub_localization_valid;
  int lost_bad_scan_limit = 5;
  int tracking_min_matched_points = 50;
  double tracking_max_mean_abs_point_plane = 0.5;
  double tracking_min_normal_eigenvalue = 14.0;
  double local_map_load_radius = 30.0;
  double local_map_unload_radius = 40.0;
  int local_map_max_load_per_scan = 1;
  int local_map_preload_limit = 20;
  TrackingQuality last_tracking_quality;
  ros::Subscriber sub_initialpose;
  InitialPoseRequestBuffer initial_pose_requests;
  bool rviz_initialpose_enabled = true;
  bool imu_orientation_ready = false;
  bool direct_validation_active = false;
  std::uint64_t active_direct_generation = 0;
  std::uint64_t handled_direct_generation = 0;
  int direct_validation_streak = 0;
  int direct_pose_validation_scans = 3;
  double direct_pose_validation_timeout_sec = 5.0;
  double direct_validation_started_sec = 0.0;
  bool use_prior_pose_for_btc = true;
  Eigen::Vector2d btc_prior_xy = Eigen::Vector2d::Zero();
  double btc_search_radius = 30.0;
  bool btc_expand_on_failure = true;
  double btc_max_search_radius = 100.0;
  bool btc_fallback_to_global = true;
  double alignment_max_translation_m = 1000000.0;
  double alignment_max_rotation_deg = 180.0;
  std::atomic<bool> frontend_reset_requested{false};
  bool lost_recovery_pending = false;
  int is_save_map;

  VOXEL_SLAM(ros::NodeHandle &n)
  {
    double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
    double btc_prior_x = 0.0, btc_prior_y = 0.0;
    vector<double> vecR(9), vecT(3);
    scanPoses = new vector<ScanPose*>();
    keyframes = new vector<Keyframe*>();
    
    string lid_topic, imu_topic;
    n.param<string>("General/lid_topic", lid_topic, "/livox/lidar");
    n.param<string>("General/imu_topic", imu_topic, "/livox/imu");
    n.param<string>("General/bagname", bagname, "site3_handheld_4");
    n.param<string>("General/save_path", savepath, "");
    n.param<string>("General/mode", operation_mode, "mapping");
    n.param<string>("General/map_export_path", map_export_path, "");
    n.param<string>("Localization/map_path", localization_map_path, "");
    n.param<double>("Localization/local_map_load_radius",
                    local_map_load_radius, 30.0);
    n.param<double>("Localization/local_map_unload_radius",
                    local_map_unload_radius, 40.0);
    n.param<int>("Localization/local_map_max_load_per_scan",
                 local_map_max_load_per_scan, 1);
    n.param<int>("Localization/local_map_preload_limit",
                 local_map_preload_limit, 20);
    n.param<bool>("Localization/rviz_initialpose_enabled",
                  rviz_initialpose_enabled, true);
    n.param<int>("Localization/direct_pose_validation_scans",
                 direct_pose_validation_scans, 3);
    n.param<double>("Localization/direct_pose_validation_timeout_sec",
                    direct_pose_validation_timeout_sec, 5.0);
    n.param<int>("Localization/lost_bad_scan_limit",
                 lost_bad_scan_limit, 5);
    n.param<int>("Localization/tracking_min_matched_points",
                 tracking_min_matched_points, 50);
    n.param<double>("Localization/tracking_max_mean_abs_point_plane",
                    tracking_max_mean_abs_point_plane, 0.5);
    n.param<double>("Localization/tracking_min_normal_eigenvalue",
                    tracking_min_normal_eigenvalue, 14.0);
    n.param<bool>("Localization/use_prior_pose_for_btc",
                  use_prior_pose_for_btc, true);
    n.param<double>("Localization/btc_prior_pose/x",
                    btc_prior_x, 0.0);
    n.param<double>("Localization/btc_prior_pose/y",
                    btc_prior_y, 0.0);
    btc_prior_xy << btc_prior_x, btc_prior_y;
    n.param<double>("Localization/btc_search_radius",
                    btc_search_radius, 30.0);
    n.param<bool>("Localization/btc_expand_on_failure",
                  btc_expand_on_failure, true);
    n.param<double>("Localization/btc_max_search_radius",
                    btc_max_search_radius, 100.0);
    n.param<bool>("Localization/btc_fallback_to_global",
                  btc_fallback_to_global, true);
    n.param<double>("Localization/alignment_max_translation_m",
                    alignment_max_translation_m, 1000000.0);
    n.param<double>("Localization/alignment_max_rotation_deg",
                    alignment_max_rotation_deg, 180.0);
    n.param<int>("General/lidar_type", feat.lidar_type, 0);
    n.param<double>("General/blind", feat.blind, 0.1);
    n.param<int>("General/point_filter_num", feat.point_filter_num, 3);
    n.param<vector<double>>("General/extrinsic_tran", vecT, vector<double>());
    n.param<vector<double>>("General/extrinsic_rota", vecR, vector<double>());
    n.param<int>("General/is_save_map", is_save_map, 0);

    if(operation_mode != "mapping" && operation_mode != "localization")
    {
      ROS_FATAL_STREAM("Unknown General/mode '" << operation_mode
                       << "'; expected 'mapping' or 'localization'.");
      throw std::invalid_argument("unknown General/mode");
    }
    if(operation_mode != "mapping" && !map_export_path.empty())
    {
      ROS_FATAL_STREAM("General/map_export_path is only valid in mapping mode.");
      throw std::invalid_argument(
          "map_export_path is only valid in mapping mode");
    }
    if(operation_mode == "localization" &&
       (local_map_load_radius <= 0.0 ||
        local_map_unload_radius < local_map_load_radius ||
        local_map_max_load_per_scan < 0 || local_map_preload_limit <= 0 ||
        direct_pose_validation_scans <= 0 ||
        direct_pose_validation_timeout_sec <= 0.0 ||
        lost_bad_scan_limit <= 0 || btc_search_radius <= 0.0 ||
        tracking_min_matched_points <= 0 ||
        tracking_max_mean_abs_point_plane <= 0.0 ||
        tracking_min_normal_eigenvalue <= 0.0 ||
        btc_max_search_radius < btc_search_radius ||
        alignment_max_translation_m <= 0.0 ||
        alignment_max_rotation_deg <= 0.0 ||
        alignment_max_rotation_deg > 180.0 ||
        !btc_prior_xy.array().isFinite().all() ||
        !std::isfinite(btc_search_radius) ||
        !std::isfinite(btc_max_search_radius) ||
        !std::isfinite(tracking_max_mean_abs_point_plane) ||
        !std::isfinite(tracking_min_normal_eigenvalue) ||
        !std::isfinite(alignment_max_translation_m) ||
        !std::isfinite(alignment_max_rotation_deg)))
    {
      ROS_FATAL("Localization map-cache or direct-validation settings are invalid.");
      throw std::invalid_argument("invalid localization settings");
    }

    sub_imu = n.subscribe(imu_topic, 80000, imu_handler);
    if(feat.lidar_type == LIVOX)
      sub_pcl = n.subscribe<livox_ros_driver::CustomMsg>(lid_topic, 1000, pcl_handler);
    else
      sub_pcl = n.subscribe<sensor_msgs::PointCloud2>(lid_topic, 1000, pcl_handler);
    if(operation_mode == "localization" && rviz_initialpose_enabled)
      sub_initialpose = n.subscribe(
          "/initialpose", 5, &VOXEL_SLAM::initialpose_callback, this);
    if(operation_mode == "localization")
    {
      localization_machine = LocalizationStateMachine(lost_bad_scan_limit);
      pub_localization_state = n.advertise<std_msgs::String>(
          "/localization/state", 1, true);
      pub_localization_valid = n.advertise<std_msgs::Bool>(
          "/localization/valid", 1, true);
      publish_localization_status();
    }
    odom_ekf.imu_topic = imu_topic;

    n.param<double>("Odometry/cov_gyr", cov_gyr, 0.1);
    n.param<double>("Odometry/cov_acc", cov_acc, 0.1);
    n.param<double>("Odometry/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("Odometry/rdw_acc", rand_walk_acc, 1e-4);
    n.param<double>("Odometry/down_size", down_size, 0.1);
    n.param<double>("Odometry/dept_err", dept_err, 0.02);
    n.param<double>("Odometry/beam_err", beam_err, 0.05);
    n.param<double>("Odometry/voxel_size", voxel_size, 1);
    n.param<double>("Odometry/min_eigen_value", min_eigen_value, 0.0025);
    n.param<int>("Odometry/degrade_bound", degrade_bound, 10);
    n.param<int>("Odometry/point_notime", point_notime, 0);
    odom_ekf.point_notime = point_notime;

    feat.blind = feat.blind * feat.blind;
    odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
    odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
    odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
    odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
    odom_ekf.Lid_offset_to_IMU  << vecT[0], vecT[1], vecT[2];
    odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2],
                            vecR[3], vecR[4], vecR[5],
                            vecR[6], vecR[7], vecR[8];                
    extrin_para.R = odom_ekf.Lid_rot_to_IMU;
    extrin_para.p = odom_ekf.Lid_offset_to_IMU;
    min_point << 5, 5, 5, 5;

    n.param<int>("LocalBA/win_size", win_size, 10);
    n.param<int>("LocalBA/max_layer", max_layer, 2);
    n.param<double>("LocalBA/cov_gyr", cov_gyr, 0.1);
    n.param<double>("LocalBA/cov_acc", cov_acc, 0.1);
    n.param<double>("LocalBA/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("LocalBA/rdw_acc", rand_walk_acc, 1e-4);
    n.param<int>("LocalBA/min_ba_point", min_ba_point, 20);
    n.param<vector<double>>("LocalBA/plane_eigen_value_thre", plane_eigen_value_thre, vector<double>({1, 1, 1, 1}));
    n.param<double>("LocalBA/imu_coef", imu_coef, 1e-4);
    n.param<int>("LocalBA/thread_num", thread_num, 5);

    for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;
    // for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;

    noiseMeas.setZero(); noiseWalk.setZero();
    noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, 
                            cov_acc, cov_acc, cov_acc;
    noiseWalk.diagonal() << 
    rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, 
    rand_walk_acc, rand_walk_acc, rand_walk_acc;

    int ss = 0;
    if(access((savepath+bagname+"/").c_str(), X_OK) == -1)
    {
      string cmd = "mkdir " + savepath + bagname + "/";
      ss = system(cmd.c_str());
    }
    else
      ss = -1;

    if(ss != 0 && is_save_map == 1)
    {
      printf("The pointcloud will be saved in this run.\n");
      printf("So please clear or rename the existed folder.\n"); 
      exit(0);
    }

    sws.resize(thread_num);
    cout << "bagname: " << bagname << endl;
  }

  void initialpose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &message)
  {
    if(!rviz_initialpose_enabled || !message) return;
    const geometry_msgs::Quaternion &orientation =
        message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if(!quaternion.coeffs().array().isFinite().all() ||
       quaternion.norm() < 1e-12)
    {
      ROS_WARN("Rejected RViz initial pose with an invalid quaternion.");
      return;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const Eigen::Vector2d xy(message->pose.pose.position.x,
                             message->pose.pose.position.y);
    std::uint64_t generation = 0;
    string error;
    if(!initial_pose_requests.submit(message->header.frame_id,
                                     message->header.stamp.toSec(), xy, yaw,
                                     generation, error))
    {
      ROS_WARN_STREAM("Rejected RViz initial pose: " << error);
      return;
    }
    ROS_INFO_STREAM("Captured RViz initial pose generation " << generation);
  }

  void publish_localization_status()
  {
    if(operation_mode != "localization") return;
    std_msgs::String state_message;
    std_msgs::Bool valid_message;
    state_message.data = localization_state_name(localization_machine.state());
    valid_message.data = localization_machine.localization_valid();
    pub_localization_state.publish(state_message);
    pub_localization_valid.publish(valid_message);
  }

  LocalizationTransition handle_localization_event(LocalizationEvent event)
  {
    lock_guard<mutex> lock(localization_state_mutex);
    string warning;
    LocalizationTransition transition =
        localization_machine.handle(event, warning);
    if(!transition.accepted)
      ROS_WARN_STREAM("Ignored " << warning << " in state "
                      << localization_state_name(localization_machine.state()));
    localization_valid.store(localization_machine.localization_valid());
    publish_localization_status();
    return transition;
  }

  LocalizationState localization_state_snapshot()
  {
    lock_guard<mutex> lock(localization_state_mutex);
    return localization_machine.state();
  }

  AlignmentSource alignment_source_snapshot()
  {
    lock_guard<mutex> lock(localization_state_mutex);
    return localization_machine.alignment_source();
  }

  bool run_if_active_alignment(AlignmentSource source,
                               const std::function<void()> &action)
  {
    lock_guard<mutex> lock(localization_state_mutex);
    if(localization_machine.state() != LocalizationState::ALIGNING ||
       localization_machine.alignment_source() != source || !action)
      return false;
    action();
    return true;
  }

  void release_voxel_map(unordered_map<VOXEL_LOC, OctoTree*> &map)
  {
    vector<OctoTree*> descendants;
    for(auto &entry: map)
    {
      if(entry.second == nullptr) continue;
      entry.second->clear_slwd(sws[0]);
      entry.second->tras_ptr(descendants);
      delete entry.second;
      entry.second = nullptr;
    }
    for(OctoTree *node: descendants)
      delete node;
    map.clear();
  }

  bool install_historical_cache(const PriorMapCacheUpdate &update,
                                string &error)
  {
    error.clear();
    if(!update.world_cloud || update.world_cloud->empty())
    {
      error = "historical local-map cache is empty";
      return false;
    }

    unordered_map<VOXEL_LOC, OctoTree*> candidate_map;
    try
    {
      PVec points;
      points.reserve(update.world_cloud->size());
      for(const PointType &point: update.world_cloud->points)
      {
        pointVar value;
        value.pnt << point.x, point.y, point.z;
        value.var = Eigen::Vector3d(point.normal_x, point.normal_y,
                                    point.normal_z).asDiagonal();
        if(!value.pnt.array().isFinite().all() ||
           !value.var.array().isFinite().all())
        {
          error = "historical local-map cache contains a non-finite point";
          release_voxel_map(candidate_map);
          return false;
        }
        points.push_back(value);
      }

      cut_voxel(candidate_map, points, win_size, 0.0);
      for(auto &entry: candidate_map)
      {
        if(entry.second != nullptr)
          entry.second->finalize_fixed();
      }

      surf_map_slide.clear();
      surf_map.swap(candidate_map);
      release_voxel_map(candidate_map);
      return true;
    }
    catch(const std::bad_alloc &)
    {
      error = "out of memory while building the historical local map";
    }
    catch(const std::exception &exception)
    {
      error = string("failed to build the historical local map: ") +
              exception.what();
    }
    catch(...)
    {
      error = "unknown failure while building the historical local map";
    }
    release_voxel_map(candidate_map);
    return false;
  }

  bool rebuild_historical_local_map(const Eigen::Vector3d &center,
                                    string &error)
  {
    lock_guard<mutex> lock(historical_map_mutex);
    PriorMapCacheUpdate update;
    if(!prior_map_tracker.rebuild_local_map(
           center, local_map_load_radius, local_map_preload_limit,
           update, error))
      return false;
    return install_historical_cache(update, error);
  }

  bool update_historical_local_map(string &error)
  {
    lock_guard<mutex> lock(historical_map_mutex);
    PriorMapCacheUpdate update;
    if(!prior_map_tracker.update_loaded_keyframes(
           x_curr.p, local_map_load_radius, local_map_unload_radius,
           local_map_max_load_per_scan, update, error))
      return false;
    if(update.loaded_ids.empty() && update.unloaded_ids.empty())
      return true;
    return install_historical_cache(update, error);
  }

  void reset_historical_runtime_cache()
  {
    lock_guard<mutex> lock(historical_map_mutex);
    prior_map_tracker.reset_runtime_cache();
  }

  bool apply_frontend_global_correction(const IMUST &correction,
                                        string &error)
  {
    error.clear();
    lock_guard<mutex> state_lock(mtx_loop);
    if(global_correction_generation ==
       numeric_limits<std::uint64_t>::max())
    {
      error = "global correction generation is exhausted";
      return false;
    }
    if(win_count <= 0 || static_cast<size_t>(win_count) > x_buf.size())
    {
      error = "global correction found an invalid frontend window";
      return false;
    }

    CorrectionTargets targets;
    targets.current_state = toCorrectionState(x_curr);
    targets.rotate_gravity = (g_update == 1);
    targets.has_applied_generation = has_global_correction_generation;
    targets.last_applied_generation = global_correction_generation;
    targets.window_states.reserve(win_count);
    for(int i=0; i<win_count; ++i)
      targets.window_states.push_back(toCorrectionState(x_buf[i]));
    targets.cached_states.reserve(buf_lba2loop.size());
    for(ScanPose *pose: buf_lba2loop)
    {
      if(pose == nullptr)
      {
        error = "global correction found a null cached pose";
        return false;
      }
      targets.cached_states.push_back(toCorrectionState(pose->x));
    }

    const std::uint64_t next_generation = global_correction_generation + 1;
    if(!apply_global_correction_once(correction, next_generation, targets,
                                     error))
      return false;

    updateFromCorrectionState(targets.current_state, x_curr);
    for(int i=0; i<win_count; ++i)
      updateFromCorrectionState(targets.window_states[i], x_buf[i]);
    size_t cached_index = 0;
    for(ScanPose *pose: buf_lba2loop)
    {
      updateFromCorrectionState(targets.cached_states[cached_index], pose->x);
      ++cached_index;
    }
    has_global_correction_generation = true;
    global_correction_generation = targets.last_applied_generation;
    return true;
  }

  // Returns 1 when a newer request starts validation, 0 when there is no
  // actionable request yet, and -1 after a hard failure requiring LIO reset.
  int try_start_direct_pose_validation(double now_sec, string &error)
  {
    error.clear();
    if(!rviz_initialpose_enabled || !prior_map_ready.load() ||
       !imu_orientation_ready)
      return 0;

    const PendingInitialPose request = initial_pose_requests.snapshot();
    if(!request.valid || request.generation == handled_direct_generation ||
       request.generation == active_direct_generation)
      return 0;

    if(!initial_pose_requests.is_current(request.generation))
      return 0;
    if(!handle_localization_event(
           LocalizationEvent::DIRECT_POSE_STARTED).accepted)
    {
      error = "localization state rejected the RViz initial pose";
      handled_direct_generation = request.generation;
      return -1;
    }

    DirectPoseSeed seed;
    if(!make_direct_pose_seed(request, true, x_curr.R, x_curr.p,
                              prior_map_tracker, seed, error))
    {
      handled_direct_generation = request.generation;
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      return -1;
    }
    if(!initial_pose_requests.is_current(request.generation))
      return 0;

    if(!rebuild_historical_local_map(seed.T_W_C.t, error))
    {
      handled_direct_generation = request.generation;
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      return -1;
    }
    if(!initial_pose_requests.is_current(request.generation))
      return 0;

    IMUST correction;
    correction.R = seed.T_W_O.R;
    correction.p = seed.T_W_O.t;
    bool correction_succeeded = false;
    const bool request_was_current = initial_pose_requests.run_if_current(
        request.generation, [&]() {
          correction_succeeded =
              apply_frontend_global_correction(correction, error);
        });
    if(!request_was_current)
      return 0;
    if(!correction_succeeded)
    {
      handled_direct_generation = request.generation;
      reset_historical_runtime_cache();
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      return -1;
    }

    direct_validation_active = true;
    active_direct_generation = request.generation;
    handled_direct_generation = request.generation;
    direct_validation_streak = 0;
    direct_validation_started_sec = now_sec;
    ROS_INFO_STREAM("Started direct-pose validation for RViz generation "
                    << request.generation << " using height keyframe "
                    << seed.height_keyframe_id);
    return 1;
  }

  // The point-to-plane alignment for odometry
  bool lio_state_estimation(PVecPtr pptr, TrackingQuality *quality = nullptr,
                            double minimum_normal_eigenvalue = 14.0)
  {
    TrackingQuality candidate_quality;
    IMUST x_prop = x_curr;

    const int num_max_iter = 4;
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();
    int rematch_num = 0;
    int match_num = 0;
    double absolute_residual_sum = 0.0;
    bool solve_succeeded = true;

    int psize = pptr->size();
    vector<OctoTree*> octos;
    octos.resize(psize, nullptr);

    Eigen::Matrix3d nnt; 
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();
    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      absolute_residual_sum = 0.0;
      nnt.setZero();

      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane* pla = nullptr;
        int flag = 0;
        if(octos[i] != nullptr && octos[i]->inside(wld))
        {
          double max_prob = 0;
          flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
        }
        else
        {
          flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
        }

        if(flag)
        // if(pla != nullptr)
        {
          Plane &pp = *pla;
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = pp.normal.dot(wld - pp.center);
          absolute_residual_sum += std::abs(resi);

          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
          jac.tail(3) = pp.normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += pp.normal * pp.normal.transpose();
          match_num++;
        }

      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      if(!K_1.array().isFinite().all() ||
         !solution.array().isFinite().all())
      {
        solve_succeeded = false;
        break;
      }

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) 
        flg_EKF_converged = true;

      if(flg_EKF_converged || ((rematch_num==0) && (iterCount==num_max_iter-2)))
      {       
        rematch_num++;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();
    // printf("eva %d: %lf\n", match_num, evalue[0]);
    candidate_quality.matched_points = match_num;
    candidate_quality.mean_abs_point_plane =
        match_num > 0
            ? absolute_residual_sum / static_cast<double>(match_num)
            : std::numeric_limits<double>::infinity();
    candidate_quality.min_normal_eigenvalue = evalue[0];
    candidate_quality.finite =
        solve_succeeded && x_curr.R.array().isFinite().all() &&
        x_curr.p.array().isFinite().all() &&
        x_curr.v.array().isFinite().all() &&
        x_curr.cov.array().isFinite().all() &&
        std::isfinite(candidate_quality.mean_abs_point_plane) &&
        std::isfinite(candidate_quality.min_normal_eigenvalue);
    candidate_quality.solve_succeeded =
        solve_succeeded && match_num > 0;
    if(quality != nullptr)
      *quality = candidate_quality;

    return candidate_quality.solve_succeeded &&
           candidate_quality.finite &&
           candidate_quality.min_normal_eigenvalue >=
               minimum_normal_eigenvalue;
  }

  // The point-to-plane alignment for initialization
  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr)
  {
    static pcl::KdTreeFLANN<PointType> kd_map;
    if(pl_tree->size() < 100)
    {
      for(pointVar pv: *pptr)
      {
        PointType pp;
        pv.pnt = x_curr.R * pv.pnt + x_curr.p;
        pp.x = pv.pnt[0]; pp.y = pv.pnt[1]; pp.z = pv.pnt[2];
        pl_tree->push_back(pp);
      }
      kd_map.setInputCloud(pl_tree);
      return;
    }

    const int num_max_iter = 4;
    IMUST x_prop = x_curr;
    int psize = pptr->size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();

    double max_dis = 2*2;
    vector<float> sqdis(NMATCH); vector<int> nearInd(NMATCH);
    PLV(3) vecs(NMATCH);
    int rematch_num = 0;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    vector<double> ds(psize, -1);
    PLV(3) directs(psize);
    bool refind = true;

    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      int valid = 0;
      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if(refind)
        {
          PointType apx;
          apx.x = wld[0]; apx.y = wld[1]; apx.z = wld[2];
          kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

          Eigen::Matrix<double, NMATCH, 3> A;
          for(int i=0; i<NMATCH; i++)
          {
            PointType &pp = pl_tree->points[nearInd[i]];
            A.row(i) << pp.x, pp.y, pp.z;
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for(int i=0; i<NMATCH; i++)
          {
            if(fabs(direct.dot(A.row(i)) + 1.0) > 0.1) 
              check_flag = true;
          }

          if(check_flag) 
          {
            ds[i] = -1;
            continue;
          }
          
          double d = 1.0 / direct.norm();
          // direct *= d;
          ds[i] = d;
          directs[i] = direct * d;
        }

        if(ds[i] >= 0)
        {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, 6, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
          valid++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
      {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if(iterCount == num_max_iter-2 && !flg_EKF_converged)
      {
        refind = true;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    double tt1 = ros::Time::now().toSec();
    for(pointVar pv: *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType ap;
      ap.x = pv.pnt[0]; ap.y = pv.pnt[1]; ap.z = pv.pnt[2];
      pl_tree->push_back(ap);
    }
    down_sampling_voxel(*pl_tree, 0.5);
    kd_map.setInputCloud(pl_tree);
    double tt2 = ros::Time::now().toSec();
  }

  // After detecting loop closure, refine current map and states
  void loop_update()
  {
    printf("loop update: %zu\n", sws[0].size());
    double t1 = ros::Time::now().toSec();

    string correction_error;
    if(!apply_frontend_global_correction(dx, correction_error))
    {
      ROS_ERROR("global correction rejected: %s", correction_error.c_str());
      loop_detect = 0;
      return;
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      // octos_release.push_back(iter->second);
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second; iter->second = nullptr;
    }
    surf_map.clear(); surf_map_slide.clear();
    surf_map = map_loop;
    map_loop.clear();

    printf("scanPoses: %zu %zu %zu %d %d %zu\n", scanPoses->size(), buf_lba2loop.size(), x_buf.size(), win_base, win_count, sws[0].size());
    int blsize = scanPoses->size();
    PointType ap = pcl_path[0];
    pcl_path.clear();
    
    for(int i=0; i<blsize; i++)
    {
      ap.x = scanPoses->at(i)->x.p[0];
      ap.y = scanPoses->at(i)->x.p[1];
      ap.z = scanPoses->at(i)->x.p[2];
      pcl_path.push_back(ap);
    }

    for(ScanPose *bl: buf_lba2loop)
    {
      ap.x = bl->x.p[0];
      ap.y = bl->x.p[1];
      ap.z = bl->x.p[2];
      pcl_path.push_back(ap);
    }
    
    for(int i=0; i<win_count; i++)
    {
      IMUST &x = x_buf[i];
      // PointType ap;
      ap.x = x.p[0]; ap.y = x.p[1]; ap.z = x.p[2];
      pcl_path.push_back(ap);
    }

    pub_pl_func(pcl_path, pub_curr_path);

    x_curr.R = x_buf[win_count-1].R;
    x_curr.p = x_buf[win_count-1].p;
    x_curr.g = x_buf[win_count-1].g;
    
    for(int i=0; i<win_size; i++)
      mp[i] = i;

    for(ScanPose *bl: buf_lba2loop)
    {
      IMUST xx = bl->x;
      PVec pvec_tem = *(bl->pvec);
      for(pointVar &pv: pvec_tem)
        pv.pnt = xx.R * pv.pnt + xx.p;
      cut_voxel(surf_map, pvec_tem, win_size, 0);
    }
    
    PLV(3) pwld;
    for(int i=0; i<win_count; i++)
    {
      pwld.clear();
      for(pointVar &pv: *pvec_buf[i])
        pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      iter->second->recut(win_count, x_buf, sws[0]);

    if(g_update == 1) g_update = 2;
    loop_detect = 0;
    double t2 = ros::Time::now().toSec();
    printf("loop head: %lf %zu\n", t2 - t1, sws[0].size());
  }

  // load the previous keyframe in the local voxel map
  void keyframe_loading(double jour)
  {
    if(history_kfsize <= 0) return;
    double tt1 = ros::Time::now().toSec();
    PointType ap_curr;
    ap_curr.x = x_curr.p[0];
    ap_curr.y = x_curr.p[1];
    ap_curr.z = x_curr.p[2];
    vector<int> vec_idx;
    vector<float> vec_dis;
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);

    for(int id: vec_idx)
    {
      int ord_kf = pl_kdmap->points[id].curvature;
      if(keyframes->at(id)->exist)
      {
        Keyframe &kf = *(keyframes->at(id));
        IMUST &xx = kf.x0;
        PVec pvec; pvec.reserve(kf.plptr->size());

        pointVar pv; pv.var.setZero();
        int plsize = kf.plptr->size();
        // for(int j=0; j<plsize; j+=2)
        for(int j=0; j<plsize; j++)
        {
          PointType ap = kf.plptr->points[j];
          pv.pnt << ap.x, ap.y, ap.z;
          pv.pnt = xx.R * pv.pnt + xx.p;
          pvec.push_back(pv);
        }

        cut_voxel(surf_map, pvec, win_size, jour);
        kf.exist = 0;
        history_kfsize--;
        break;
      }
    }
    
  }

  int initialization(deque<sensor_msgs::Imu::Ptr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) &pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    static vector<double> beg_times;
    static vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;

    pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));
    if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      return 0;

    if(win_count == 0)
      imupre_scale_gravity = odom_ekf.scale_gravity;

    PVecPtr pptr(new PVec);
    double downkd = down_size >= 0.5 ? down_size : 0.5;
    down_sampling_voxel(*pcl_curr, downkd);
    var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
    lio_state_estimation_kdtree(pptr);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, sessionNames.size()-1, pcl_path);

    if(win_count > 1)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
      imu_pre_buf[win_count-2]->push_imu(imus);
    }

    pcl::PointCloud<PointType> pl_mid = *orig;
    down_sampling_close(*orig, down_size);
    if(orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size / 2);
    }

    sort(orig->begin(), orig->end(), [](PointType &x, PointType &y)
    {return x.curvature < y.curvature;});

    pl_origs.push_back(orig);
    beg_times.push_back(odom_ekf.pcl_beg_time);
    vec_imus.push_back(imus);

    int is_success = 0;
    if(win_count >= win_size)
    {
      is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf, extrin_para);

      if(is_success == 0)
        return -1;
      return 1;
    }
    return 0;
  }

  void system_reset(deque<sensor_msgs::Imu::Ptr> &imus)
  {
    string epoch_error;
    if(!frontend_session_epoch.advance(epoch_error))
    {
      ROS_FATAL_STREAM("Cannot reset frontend session: " << epoch_error);
      ros::shutdown();
      return;
    }
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear(); surf_map_slide.clear();

    x_curr.setZero();
    x_curr.p = Eigen::Vector3d(0, 0, 30);
    odom_ekf.mean_acc.setZero();
    odom_ekf.init_num = 0;
    odom_ekf.IMU_init(imus);
    x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;
    imu_orientation_ready = false;

    for(int i=0; i<imu_pre_buf.size(); i++)
      delete imu_pre_buf[i];
    x_buf.clear(); pvec_buf.clear(); imu_pre_buf.clear();
    pl_tree->clear();

    for(int i=0; i<win_size; i++)
      mp[i] = i;
    win_base = 0; win_count = 0; pcl_path.clear();
    pub_pl_func(pcl_path, pub_cmap);
    ROS_WARN("Reset");
  }

  // After local BA, update the map and marginalize the points of oldest scan
  // multi means multiple thread
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, double jour, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<SlideWindow*> &sw)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    // {
    //   iter->second->jour = jour;
    //   iter->second->margi(win_count, 1, xs, voxopt);
    //   if(iter->second->isexist)
    //     iter++;
    //   else
    //   {
    //     iter->second->clear_slwd(sw);
    //     feat_map.erase(iter++);
    //   }
    // }
    // return;

    int thd_num = thread_num;
    vector<vector<OctoTree*>*> octs;
    for(int i=0; i<thd_num; i++) 
      octs.push_back(new vector<OctoTree*>());

    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      iter->second->jour = jour;
      octs[cnt]->push_back(iter->second);
      if(octs[cnt]->size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto margi_func = [](int win_cnt, vector<OctoTree*> *oct, vector<IMUST> xxs, LidarFactor &voxhess)
    {
      for(OctoTree *oc: *oct)
      {
        oc->margi(win_cnt, 1, xxs, voxhess);
      }
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
    }
    
    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        margi_func(win_count, octs[i], xs, voxopt);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    {
      if(iter->second->isexist)
        iter++;
      else
      {
        iter->second->clear_slwd(sw);
        feat_map.erase(iter++);
      }
    }

    for(int i=0; i<thd_num; i++)
      delete octs[i];

  }

  // Determine the plane and recut the voxel map in octo-tree
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<vector<SlideWindow*>> &sws)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    // {
    //   iter->second->recut(win_count, xs, sws[0]);
    //   iter->second->tras_opt(voxopt);
    // }

    int thd_num = thread_num;
    vector<vector<OctoTree*>> octss(thd_num);
    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      octss[cnt].push_back(iter->second);
      if(octss[cnt].size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto recut_func = [](int win_count, vector<OctoTree*> &oct, vector<IMUST> xxs, vector<SlideWindow*> &sw)
    {
      for(OctoTree *oc: oct)
        oc->recut(win_count, xxs, sw);
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));
    }

    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        recut_func(win_count, octss[i], xs, sws[i]);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(int i=1; i<sws.size(); i++)
    {
      sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
      sws[i].clear();
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
      iter->second->tras_opt(voxopt);

  }

  // The main thread of odometry and local mapping
  void thd_odometry_localmapping(ros::NodeHandle &n)
  {
    PLV(3) pwld;
    double down_sizes[3] = {0.1, 0.2, 0.4};
    Eigen::Vector3d last_pos(0, 0 ,0);
    double jour = 0;
    int counter = 0;

    pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
    int motion_init_flag = 1;
    pl_tree.reset(new pcl::PointCloud<PointType>());
    vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    vector<double> beg_times;
    vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;
    bool release_flag = false;
    int degrade_cnt = 0;
    LidarFactor voxhess(win_size);
    const int mgsize = 1;
    Eigen::MatrixXd hess;
    while(n.ok())
    {
      ros::spinOnce();
      if(loop_detect == 1)
      {
        loop_update(); last_pos = x_curr.p; jour = 0;
      }
      
      bool finish_requested = false;
      n.param<bool>("finish", finish_requested, false);
      is_finish.store(finish_requested);
      if(finish_requested)
      {
        break;
      }

      deque<sensor_msgs::Imu::Ptr> imus;
      if(!sync_packages(pcl_curr, imus, odom_ekf))
      {
        if(octos_release.size() != 0)
        {
          int msize = octos_release.size();
          if(msize > 1000) msize = 1000;
          for(int i=0; i<msize; i++)
          {
            delete octos_release.back();
            octos_release.pop_back();
          }
          malloc_trim(0);
        }
        else if(release_flag)
        {
          release_flag = false;
          vector<OctoTree*> octos;
          for(auto iter=surf_map.begin(); iter!=surf_map.end();)
          {
            int dis = jour - iter->second->jour;
            if(dis < 700)
            // if(dis < 200)
            {
              iter++;
            }
            else
            {
              octos.push_back(iter->second);
              iter->second->tras_ptr(octos);
              surf_map.erase(iter++);
            }
          }
          int ocsize = octos.size();
          for(int i=0; i<ocsize; i++)
            delete octos[i];
          octos.clear();
          malloc_trim(0);
        }
        else if(sws[0].size() > 10000)
        {
          for(int i=0; i<500; i++)
          {
            delete sws[0].back();
            sws[0].pop_back();
          }
          malloc_trim(0);
        }

        sleep(0.001);
        continue;
      }

      unique_lock<mutex> localization_pipeline_lock;
      if(operation_mode == "localization")
        localization_pipeline_lock =
            unique_lock<mutex>(frontend_pipeline_mutex);
      if(frontend_reset_requested.exchange(false))
      {
        system_reset(imus);
        motion_init_flag = 1;
        reset_flag = 1;
        continue;
      }
      if(lost_recovery_pending)
      {
        handle_localization_event(LocalizationEvent::RECOVERY_STARTED);
        lost_recovery_pending = false;
      }

      static int first_flag = 1;
      if (first_flag)
      {
        pcl::PointCloud<PointType> pl;
        pub_pl_func(pl, pub_pmap);
        pub_pl_func(pl, pub_prev_path);
        first_flag = 0;
      }

      double t0 = ros::Time::now().toSec();
      double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0, t7=0, t8=0;

      if(motion_init_flag)
      {
        int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

        if(init == 1)
        {
          motion_init_flag = 0;
          imu_orientation_ready = true;
        }
        else
        {
          if(init == -1)
            system_reset(imus);
          continue;
        }
      }
      else
      {
        if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
          continue;

        string direct_pose_error;
        const int direct_start = try_start_direct_pose_validation(
            ros::Time::now().toSec(), direct_pose_error);
        if(direct_start < 0)
        {
          ROS_WARN_STREAM("Direct-pose initialization failed: "
                          << direct_pose_error);
          direct_validation_active = false;
          active_direct_generation = 0;
          reset_historical_runtime_cache();
          system_reset(imus);
          motion_init_flag = 1;
          reset_flag = 1;
          continue;
        }

        pcl::PointCloud<PointType> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size);

        if(pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size / 2);
        }

        PVecPtr pptr(new PVec);
        var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

        TrackingQuality scan_quality;
        if(operation_mode == "localization" &&
           (localization_valid.load() || direct_validation_active) &&
           prior_map_ready.load())
        {
          string cache_error;
          bool tracking_good = false;
          if(update_historical_local_map(cache_error))
          {
            const bool solver_good =
                lio_state_estimation(pptr, &scan_quality,
                                     tracking_min_normal_eigenvalue);
            tracking_good =
                solver_good &&
                scan_quality.matched_points >= tracking_min_matched_points &&
                scan_quality.mean_abs_point_plane <=
                    tracking_max_mean_abs_point_plane &&
                scan_quality.min_normal_eigenvalue >=
                    tracking_min_normal_eigenvalue;
          }
          else
          {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "Historical local-map update failed: " << cache_error);
          }
          last_tracking_quality = scan_quality;
          if(tracking_good)
          {
            if(degrade_cnt > 0) degrade_cnt--;
          }
          else
          {
            degrade_cnt++;
          }

          if(direct_validation_active)
          {
            if(tracking_good)
              direct_validation_streak++;
            else
              direct_validation_streak = 0;

            if(direct_validation_streak >= direct_pose_validation_scans)
            {
              direct_validation_active = false;
              handle_localization_event(
                  LocalizationEvent::DIRECT_POSE_VALIDATED);
              ROS_INFO_STREAM(
                  "RViz direct pose validated after "
                  << direct_validation_streak << " consecutive scans.");
            }
            else if(ros::Time::now().toSec() -
                        direct_validation_started_sec >
                    direct_pose_validation_timeout_sec)
            {
              ROS_WARN("RViz direct-pose validation timed out; resetting temporary LIO.");
              direct_validation_active = false;
              active_direct_generation = 0;
              handle_localization_event(
                  LocalizationEvent::DIRECT_POSE_FAILED);
              reset_historical_runtime_cache();
              system_reset(imus);
              motion_init_flag = 1;
              reset_flag = 1;
              continue;
            }
          }

          if(!direct_validation_active && localization_valid.load())
          {
            LocalizationEvent tracking_event =
                !scan_quality.finite
                    ? LocalizationEvent::TRACKING_NONFINITE
                    : (tracking_good ? LocalizationEvent::TRACKING_GOOD
                                     : LocalizationEvent::TRACKING_BAD);
            const LocalizationTransition tracking_transition =
                handle_localization_event(tracking_event);
            if(tracking_transition.cleanup_requested)
            {
              ROS_WARN("Localization entered LOST; clearing current-session state.");
              active_direct_generation = 0;
              direct_validation_active = false;
              reset_historical_runtime_cache();
              system_reset(imus);
              motion_init_flag = 1;
              reset_flag = 1;
              lost_recovery_pending = true;
              continue;
            }
          }

          pwld.clear();
          pvec_update(pptr, x_curr, pwld);
          ResultOutput::instance().pub_localtraj(
              pwld, jour, x_curr, sessionNames.size()-1, pcl_path);

          // Historical localization is read-only: the current scan is never
          // inserted into surf_map or any sliding-window map.
          continue;
        }

        if(lio_state_estimation(pptr, &scan_quality))
        {
          if(degrade_cnt > 0) degrade_cnt--;
        }
        else
          degrade_cnt++;
        last_tracking_quality = scan_quality;

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, sessionNames.size()-1, pcl_path);

        t1 = ros::Time::now().toSec();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        if(win_count > 1)
        {
          imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
          imu_pre_buf[win_count-2]->push_imu(imus);
        }
        
        keyframe_loading(jour);
        voxhess.clear(); voxhess.win_size = win_size;

        // cut_voxel(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws[0]);
        cut_voxel_multi(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws);
        t2 = ros::Time::now().toSec();

        multi_recut(surf_map_slide, win_count, x_buf, voxhess, sws);
        t3 = ros::Time::now().toSec();

        if(degrade_cnt > degrade_bound)
        {
          degrade_cnt = 0;
          system_reset(imus);

          last_pos = x_curr.p; jour = 0;

          mtx_loop.lock();
          buf_lba2loop_tem.swap(buf_lba2loop);
          mtx_loop.unlock();
          reset_flag = 1;

          motion_init_flag = 1;
          history_kfsize = 0;

          continue;
        }
      }

      if(win_count >= win_size)
      {
        t4 = ros::Time::now().toSec();
        
        if(g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);
          printf("g update: %lf %lf %lf: %lf\n", x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm());
          g_update = 0;
          x_curr.g = x_buf[win_count-1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }

        ScanPose *bl = new ScanPose(x_buf[0], pvec_buf[0]);
        bl->v6 = hess.block<6, 6>(0, DIM).diagonal();
        for(int i=0; i<6; i++) bl->v6[i] = 1.0 / fabs(bl->v6[i]);
        mtx_loop.lock();
        buf_lba2loop.push_back(bl);
        mtx_loop.unlock();

        x_curr.R = x_buf[win_count-1].R;
        x_curr.p = x_buf[win_count-1].p;
        t5 = ros::Time::now().toSec();

        ResultOutput::instance().pub_localmap(mgsize, sessionNames.size()-1, pvec_buf, x_buf, pcl_path, win_base, win_count);

        multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
        t6 = ros::Time::now().toSec();

        if((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if(spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        if(is_save_map)
        {
          for(int i=0; i<mgsize; i++)
            FileReaderWriter::instance().save_pcd(pvec_buf[i], x_buf[i], win_base + i, savepath + bagname);
        }

        for(int i=0; i<win_size; i++)
        {
          mp[i] += mgsize;
          if(mp[i] >= win_size) mp[i] -= win_size;
        }

        for(int i=mgsize; i<win_count; i++)
        {
          x_buf[i-mgsize] = x_buf[i];
          PVecPtr pvec_tem = pvec_buf[i-mgsize];
          pvec_buf[i-mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for(int i=win_count-mgsize; i<win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();

          delete imu_pre_buf.front();
          imu_pre_buf.pop_front();
        }

        win_base += mgsize; win_count -= mgsize;
      }
      
      double t_end = ros::Time::now().toSec();
      double mem = get_memory();
      // printf("%d: %.4lf: %.4lf %.4lf %.4lf %.4lf %.4lf %.2lfGb %.1lf\n", win_base+win_count, t_end-t0, t1-t0, t2-t1, t3-t2, t5-t4, t6-t5, mem, jour);

      // printf("%d: %lf %lf %lf\n", win_base + win_count, x_curr.p[0], x_curr.p[1], x_curr.p[2]);
    }

    vector<OctoTree *> octos;
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for(int i=0; i<octos.size(); i++)
      delete octos[i];
    octos.clear();

    for(int i=0; i<sws[0].size(); i++)
      delete sws[0][i];
    sws[0].clear();
    malloc_trim(0);
  }

  const SavedKeyframe *find_localization_keyframe(int id) const
  {
    for(const SavedKeyframe &keyframe: localization_map.keyframes)
      if(keyframe.id == id) return &keyframe;
    return nullptr;
  }

  void thd_localization_btc(ros::NodeHandle &n,
                            ConfigSetting &config_setting,
                            double jud_threshold, double icp_eigval)
  {
    STDescManager history_manager(config_setting);
    string error;
    if(!restore_btc_database(localization_map.btc_frames, history_manager,
                             error))
    {
      ROS_FATAL_STREAM("Failed to restore localization BTC database: "
                       << error);
      ros::shutdown();
      return;
    }

    BtcRelocalizer relocalizer;
    if(!relocalizer.initialize(localization_map, prior_map_tracker, error))
    {
      ROS_FATAL_STREAM("Failed to initialize BTC relocalizer: " << error);
      ros::shutdown();
      return;
    }

    BtcRelocalizationOptions relocalization_options;
    relocalization_options.use_prior_position = use_prior_pose_for_btc;
    relocalization_options.prior_xy = btc_prior_xy;
    relocalization_options.initial_radius = btc_search_radius;
    relocalization_options.expand_on_failure = btc_expand_on_failure;
    relocalization_options.maximum_radius = btc_max_search_radius;
    relocalization_options.fallback_to_global = btc_fallback_to_global;

    SessionAlignmentOptions alignment_options;
    alignment_options.max_translation_correction_m =
        alignment_max_translation_m;
    alignment_options.max_rotation_correction_deg =
        alignment_max_rotation_deg;

    deque<ScanPose*> fusion_window;
    vector<SavedPose> current_odometry;
    int query_frame_id = 0;

    auto delete_pose = [](ScanPose *pose) {
      if(pose == nullptr) return;
      pose->pvec.reset();
      delete pose;
    };
    auto clear_local_session = [&]() {
      for(ScanPose *pose: fusion_window) delete_pose(pose);
      fusion_window.clear();
      current_odometry.clear();
      lock_guard<mutex> lock(mtx_loop);
      for(ScanPose *pose: buf_lba2loop) delete_pose(pose);
      buf_lba2loop.clear();
      for(ScanPose *pose: buf_lba2loop_tem) delete_pose(pose);
      buf_lba2loop_tem.clear();
    };

    while(n.ok())
    {
      if(reset_flag.exchange(0) == 1)
      {
        clear_local_session();
      }
      if(is_finish.load()) break;
      const LocalizationState state = localization_state_snapshot();
      if(state == LocalizationState::IN_MAP_TRACKING &&
         (!fusion_window.empty() || !current_odometry.empty()))
        clear_local_session();
      if(state != LocalizationState::LOCAL_ODOMETRY)
      {
        ros::Duration(0.01).sleep();
        continue;
      }

      ScanPose *head = nullptr;
      {
        lock_guard<mutex> lock(mtx_loop);
        if(!buf_lba2loop.empty())
        {
          head = buf_lba2loop.front();
          buf_lba2loop.pop_front();
        }
      }
      if(head == nullptr)
      {
        ros::Duration(0.01).sleep();
        continue;
      }

      SavedPose odometry_pose;
      odometry_pose.stamp = head->x.t;
      odometry_pose.R = head->x.R;
      odometry_pose.t = head->x.p;
      current_odometry.push_back(odometry_pose);
      fusion_window.push_back(head);
      if(fusion_window.size() < static_cast<size_t>(win_size))
        continue;

      ScanPose *anchor = fusion_window.back();
      error.clear();
      PVec fused_points;
      for(ScanPose *pose: fusion_window)
      {
        if(pose == nullptr || !pose->pvec)
        {
          error = "BTC fusion window contains an invalid scan pose";
          break;
        }
        const Eigen::Matrix3d delta_R =
            anchor->x.R.transpose() * pose->x.R;
        const Eigen::Vector3d delta_p =
            anchor->x.R.transpose() * (pose->x.p - anchor->x.p);
        for(pointVar point: *pose->pvec)
        {
          point.pnt = delta_R * point.pnt + delta_p;
          point.var = delta_R * point.var * delta_R.transpose();
          fused_points.push_back(point);
        }
      }
      for(ScanPose *pose: fusion_window) delete_pose(pose);
      fusion_window.clear();
      if(!error.empty())
      {
        ROS_WARN_STREAM("Skipped BTC query: " << error);
        error.clear();
        continue;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr btc_cloud(
          new pcl::PointCloud<pcl::PointXYZI>());
      btc_cloud->reserve(fused_points.size());
      for(const pointVar &point: fused_points)
      {
        pcl::PointXYZI output;
        output.x = point.pnt.x();
        output.y = point.pnt.y();
        output.z = point.pnt.z();
        output.intensity = 0.0F;
        btc_cloud->push_back(output);
      }

      STDescManager query_manager(config_setting);
      vector<STD> descriptors;
      query_manager.GenerateSTDescs(btc_cloud, descriptors, query_frame_id++);
      if(query_manager.plane_cloud_vec_.empty())
      {
        ROS_WARN("BTC query generation produced no plane cloud.");
        continue;
      }
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_planes =
          query_manager.plane_cloud_vec_.back();

      const std::uint64_t query_session_epoch =
          frontend_session_epoch.capture();
      if(!handle_localization_event(
             LocalizationEvent::BTC_STARTED).accepted)
        continue;

      BtcSearchFunction raw_search = make_stdesc_btc_search(
          history_manager, descriptors, current_planes);
      BtcSearchFunction score_checked_search =
          [raw_search, jud_threshold](const std::unordered_set<int> &allowed,
                                     BtcSearchMatch &match,
                                     string &failure_reason) {
            if(!raw_search(allowed, match, failure_reason)) return false;
            if(match.score <= jud_threshold)
            {
              failure_reason = "BTC score is below the configured threshold";
              return false;
            }
            return true;
          };
      IcpVerificationFunction icp_verifier =
          [&history_manager, current_planes, icp_eigval](
              int frame_id, Eigen::Matrix3d &R_HC, Eigen::Vector3d &t_HC,
              double &observability, string &failure_reason) {
            if(frame_id < 0 ||
               static_cast<size_t>(frame_id) >=
                   history_manager.plane_cloud_vec_.size() ||
               !history_manager.plane_cloud_vec_[frame_id])
            {
              failure_reason = "BTC ICP references an invalid history frame";
              return false;
            }
            pair<Eigen::Vector3d, Eigen::Matrix3d> transform(t_HC, R_HC);
            if(!icp_normal(*current_planes,
                           *history_manager.plane_cloud_vec_[frame_id],
                           transform, icp_eigval))
            {
              failure_reason = "BTC candidate failed local-map ICP";
              return false;
            }
            t_HC = transform.first;
            R_HC = transform.second;
            observability = icp_eigval;
            failure_reason.clear();
            return true;
          };

      const RelocalizationResult relocalization = relocalizer.relocalize(
          relocalization_options, score_checked_search, icp_verifier);
      if(is_finish.load()) break;
      if(!frontend_session_epoch.is_current(query_session_epoch))
      {
        ROS_WARN("Discarded BTC result produced by a reset frontend session.");
        if(localization_state_snapshot() == LocalizationState::RELOCALIZING ||
           (localization_state_snapshot() == LocalizationState::ALIGNING &&
            alignment_source_snapshot() == AlignmentSource::BTC))
          handle_localization_event(LocalizationEvent::BTC_FAILED);
        clear_local_session();
        continue;
      }
      if(!relocalization.success)
      {
        ROS_WARN_STREAM("BTC relocalization failed: "
                        << relocalization.failure_reason);
        handle_localization_event(LocalizationEvent::BTC_FAILED);
        continue;
      }
      if(!handle_localization_event(
             LocalizationEvent::BTC_MATCHED).accepted)
        continue;

      const SavedKeyframe *history_anchor =
          find_localization_keyframe(relocalization.anchor_keyframe_id);
      if(history_anchor == nullptr || current_odometry.empty())
      {
        error = "BTC alignment anchor is unavailable";
        handle_localization_event(LocalizationEvent::BTC_FAILED);
        continue;
      }
      SessionAlignmentResult alignment;
      if(!align_current_session_to_map(
             history_anchor->pose,
             static_cast<int>(current_odometry.size() - 1),
             relocalization.R_HC, relocalization.t_HC, current_odometry,
             alignment_options, alignment, error))
      {
        ROS_WARN_STREAM("BTC session alignment failed: " << error);
        handle_localization_event(LocalizationEvent::BTC_FAILED);
        continue;
      }
      bool correction_succeeded = false;
      {
        lock_guard<mutex> pipeline_lock(frontend_pipeline_mutex);
        if(!frontend_session_epoch.is_current(query_session_epoch))
        {
          ROS_WARN("Discarded BTC alignment produced by a reset frontend session.");
          if(localization_state_snapshot() == LocalizationState::ALIGNING &&
             alignment_source_snapshot() == AlignmentSource::BTC)
            handle_localization_event(LocalizationEvent::BTC_FAILED);
          clear_local_session();
          continue;
        }
        if(localization_state_snapshot() != LocalizationState::ALIGNING ||
           alignment_source_snapshot() != AlignmentSource::BTC)
          continue;

        if(!rebuild_historical_local_map(
               alignment.optimized_current.back().t, error))
        {
          ROS_WARN_STREAM("BTC historical-map build failed: " << error);
          handle_localization_event(LocalizationEvent::BTC_FAILED);
          frontend_reset_requested.store(true);
          continue;
        }
        IMUST correction;
        correction.R = alignment.T_W_O.R;
        correction.p = alignment.T_W_O.t;
        const bool btc_alignment_still_active = run_if_active_alignment(
            AlignmentSource::BTC, [&]() {
              correction_succeeded =
                  apply_frontend_global_correction(correction, error);
            });
        if(!btc_alignment_still_active)
          continue;
        if(correction_succeeded &&
           !handle_localization_event(
                LocalizationEvent::ALIGNMENT_APPLIED).accepted)
        {
          correction_succeeded = false;
          error = "localization state rejected the BTC alignment";
        }
      }
      if(!correction_succeeded)
      {
        ROS_WARN_STREAM("BTC frontend correction failed: " << error);
        handle_localization_event(LocalizationEvent::BTC_FAILED);
        reset_historical_runtime_cache();
        frontend_reset_requested.store(true);
        continue;
      }
      ROS_INFO_STREAM("BTC localization validated with history frame "
                      << relocalization.btc_frame_id << " at score "
                      << relocalization.score);
      clear_local_session();
    }

    clear_local_session();
  }

  // Build the pose graph in loop closure
  void build_graph(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, int cur_id, PGO_Edges &lp_edges, gtsam::noiseModel::Diagonal::shared_ptr default_noise, vector<int> &ids, vector<int> &stepsizes, int lpedge_enable)
  {
    initial.clear(); graph = gtsam::NonlinearFactorGraph();
    ids.clear();
    lp_edges.connect(cur_id, ids);

    stepsizes.clear(); stepsizes.push_back(0);
    for(int i=0; i<ids.size(); i++)
      stepsizes.push_back(stepsizes.back() + multimap_scanPoses[ids[i]]->size());
    
    for(int ii=0; ii<ids.size(); ii++)
    {
      int bsize = stepsizes[ii], id = ids[ii];
      for(int j=bsize; j<stepsizes[ii+1]; j++)
      {
        IMUST &xc = multimap_scanPoses[id]->at(j-bsize)->x;
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        initial.insert(j, pose3);
        if(j > bsize)
        {
          gtsam::Vector samv6(6);
          samv6 = multimap_scanPoses[ids[ii]]->at(j-1-bsize)->v6;
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
          add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, v6_noise);
          // add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, default_noise);
        }
      }
    }

    if(multimap_scanPoses[ids[0]]->size() != 0)
    {
      int ceil = multimap_scanPoses[ids[0]]->size();
      // if(ceil > 10) ceil = 10;
      ceil = 1;
      for(int i=0; i<ceil; i++)
      {
        Eigen::Matrix<double, 6, 1> v6_fixd;
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
        IMUST xf = multimap_scanPoses[ids[0]]->at(i)->x;
        gtsam::Pose3 pose3 = gtsam::Pose3(gtsam::Rot3(xf.R), gtsam::Point3(xf.p));
        graph.addPrior(i, pose3, fixd_noise);
      }
    }

    if(lpedge_enable == 1)
    for(PGO_Edge &edge: lp_edges.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, default_noise);
        }
      }
    }
    
  }

  // The main thread of loop clousre
  // The topDownProcess of HBA is also run here
  void thd_loop_closure(ros::NodeHandle &n)
  {
    pl_kdmap.reset(new pcl::PointCloud<PointType>);
    vector<STDescManager*> std_managers;
    PGO_Edges lp_edges;
    SavedMap captured_map;
    vector<pair<int, const Keyframe*>> captured_pose_sources;
    bool map_export_capture_failed = false;

    double jud_default = 0.45, icp_eigval = 14;
    double ratio_drift = 0.05;
    int curr_halt = 10, prev_halt = 30;
    int isHighFly = 0;
    n.param<double>("Loop/jud_default", jud_default, 0.45);
    n.param<double>("Loop/icp_eigval", icp_eigval, 14);
    n.param<double>("Loop/ratio_drift", ratio_drift, 0.05);
    n.param<int>("Loop/curr_halt", curr_halt, 10);
    n.param<int>("Loop/prev_halt", prev_halt, 30);
    n.param<int>("Loop/isHighFly", isHighFly, 0);
    ConfigSetting config_setting;
    read_parameters(n, config_setting, isHighFly);

    if(operation_mode == "localization")
    {
      if(localization_map_path.empty())
      {
        ROS_FATAL("Localization/map_path must be set in localization mode.");
        ros::shutdown();
        return;
      }
      MapPackageOptions load_options;
      load_options.win_size = win_size;
      load_options.keyframe_distance_m = 0.1;
      load_options.keyframe_angle_deg = 5.0;
      load_options.keyframe_voxel_size = voxel_size / 10.0;
      load_options.btc_config = config_setting;
      string map_error;
      if(!MapPackageIO::load(localization_map_path, localization_map,
                             load_options, map_error) ||
         !prior_map_tracker.initialize(localization_map, map_error))
      {
        ROS_FATAL_STREAM("Failed to load localization map package: "
                         << map_error);
        ros::shutdown();
        return;
      }
      handle_localization_event(LocalizationEvent::MAP_LOADED);
      prior_map_ready.store(true);
      ROS_INFO_STREAM("Loaded localization map package with "
                      << localization_map.keyframes.size()
                      << " keyframes and "
                      << localization_map.btc_frames.size()
                      << " BTC frames.");
      thd_localization_btc(n, config_setting, jud_default, icp_eigval);
      return;
    }

    vector<double> juds;
    FileReaderWriter::instance().previous_map_names(n, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, bagname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, n, sessionNames, juds, savepath, win_size);
    
    STDescManager *std_manager = new STDescManager(config_setting);
    sessionNames.push_back(bagname);
    std_managers.push_back(std_manager);
    multimap_scanPoses.push_back(scanPoses);
    multimap_keyframes.push_back(keyframes);
    juds.push_back(jud_default);
    vector<double> jours(std_managers.size(), 0);

    vector<int> relc_counts(std_managers.size(), prev_halt);
    
    deque<ScanPose*> bl_local;
    Eigen::Matrix<double, 6, 1> v6_init, v6_fixd;
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    v6_fixd << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    vector<int> ids(1, std_managers.size() - 1), stepsizes(2, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plbtc(new pcl::PointCloud<pcl::PointXYZI>);
    IMUST x_key;
    int buf_base = 0;

    while(n.ok())
    {
      if(reset_flag.exchange(0) == 1)
      {
        scanPoses->insert(scanPoses->end(), buf_lba2loop_tem.begin(), buf_lba2loop_tem.end());
        for(ScanPose *bl: buf_lba2loop_tem) bl->pvec = nullptr;
        buf_lba2loop_tem.clear();

        keyframes = new vector<Keyframe*>();
        multimap_keyframes.push_back(keyframes);
        scanPoses = new vector<ScanPose*>();
        multimap_scanPoses.push_back(scanPoses);

        bl_local.clear(); buf_base = 0; 
        std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);
        std_manager = new STDescManager(config_setting);
        std_managers.push_back(std_manager);
        relc_counts.push_back(prev_halt);
        sessionNames.push_back(bagname + to_string(sessionNames.size()));
        juds.push_back(jud_default);
        jours.push_back(0);

        bagname = sessionNames.back();
        string cmd = "mkdir " + savepath + bagname + "/";
        int ss = system(cmd.c_str());

        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids, pub_pmap);

        initial.clear(); graph = gtsam::NonlinearFactorGraph();
        ids.clear(); ids.push_back(std_managers.size()-1); 
        stepsizes.clear(); stepsizes.push_back(0); stepsizes.push_back(0);
      }

      if(is_finish.load() && buf_lba2loop.empty())
      {
        break;
      }

      if(buf_lba2loop.empty() || loop_detect == 1)
      {
        sleep(0.01); continue;
      }
      ScanPose *bl_head = nullptr;
      mtx_loop.lock();
      if(!buf_lba2loop.empty()) 
      {
        bl_head = buf_lba2loop.front();
        buf_lba2loop.pop_front();
      }
      mtx_loop.unlock();
      if(bl_head == nullptr) continue;

      int cur_id = std_managers.size() - 1;
      scanPoses->push_back(bl_head);
      bl_local.push_back(bl_head);
      IMUST xc = bl_head->x;
      gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
      int g_pos = stepsizes.back();
      initial.insert(g_pos, pose3);

      if(g_pos > 0)
      {
        gtsam::Vector samv6(scanPoses->at(buf_base-1)->v6);
        gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
        add_edge(g_pos-1, g_pos, scanPoses->at(buf_base-1)->x, xc, graph, v6_noise);
      }
      else
      {
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        graph.addPrior(0, pose3, fixd_noise);
      }

      if(buf_base == 0) x_key = xc;
      buf_base++; stepsizes.back() += 1;

      if(bl_local.size() < win_size) continue;
      double ang = Log(x_key.R.transpose() * xc.R).norm() * 57.3;
      double len = (xc.p - x_key.p).norm();
      if(ang < 5 && len < 0.1 && buf_base > win_size)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
        continue;
      }
      for(double &jour: jours)
        jour += len;
      x_key = xc;

      PVecPtr pptr(new PVec);
      for(int i=0; i<win_size; i++)
      {
        ScanPose &bl = *bl_local[i];
        Eigen::Vector3d delta_p = xc.R.transpose() * (bl.x.p - xc.p);
        Eigen::Matrix3d delta_R = xc.R.transpose() *  bl.x.R;
        for(pointVar pv: *(bl.pvec))
        {
          pv.pnt = delta_R * pv.pnt + delta_p;
          pptr->push_back(pv);
        }
      }
      for(int i=0; i<win_size; i++)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
      }

      Keyframe *smp = new Keyframe(xc);
      smp->id = buf_base - 1;
      smp->jour = jours[cur_id];
      down_sampling_pvec(*pptr, voxel_size/10, *(smp->plptr));

      plbtc->clear();
      pcl::PointXYZI ap;
      for(pointVar &pv: *pptr)
      {
        Eigen::Vector3d &wld = pv.pnt;
        ap.x = wld[0]; ap.y = wld[1]; ap.z = wld[2];
        plbtc->push_back(ap);
      }
      mtx_keyframe.lock();
      keyframes->push_back(smp);
      mtx_keyframe.unlock();

      vector<STD> stds_vec;
      std_manager->GenerateSTDescs(plbtc, stds_vec, buf_base-1);
      if(operation_mode == "mapping" && !map_export_path.empty())
      {
        string capture_error;
        SavedPose captured_pose;
        captured_pose.stamp = smp->x0.t;
        captured_pose.R = smp->x0.R;
        captured_pose.t = smp->x0.p;
        int capture_id = -1;
        bool association_ready = true;
        try
        {
          if(captured_pose_sources.size() == captured_pose_sources.capacity())
            captured_pose_sources.reserve(captured_pose_sources.size() + 1024);
        }
        catch(const std::bad_alloc&)
        {
          capture_error = "out of memory while reserving final-pose associations";
          association_ready = false;
        }
        if(!association_ready ||
           !captureAcceptedKeyframe(
               true, captured_pose, smp->plptr, stds_vec,
               std_manager->plane_cloud_vec_.back(), captured_map, capture_id,
               capture_error))
        {
          ROS_FATAL_STREAM("Map-package keyframe capture failed: "
                           << capture_error);
          map_export_capture_failed = true;
          ros::shutdown();
          break;
        }
        captured_pose_sources.push_back(make_pair(capture_id, smp));
      }
      pair<int, double> search_result(-1, 0);
      pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      vector<pair<STD, STD>> loop_std_pair;

      bool isGraph = false, isOpt = false;
      int match_num = 0;
      for(int id=0; id<=cur_id; id++)
      {
        std_managers[id]->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, std_manager->plane_cloud_vec_.back());

        if(search_result.first >= 0)
        {
          printf("Find Loop in session%d: %d %d\n", id, buf_base, search_result.first);
          printf("score: %lf\n", search_result.second);
        }

        if(search_result.first >= 0 && search_result.second > juds[id])
        {
          if(icp_normal(*(std_manager->plane_cloud_vec_.back()), *(std_managers[id]->plane_cloud_vec_[search_result.first]), loop_transform, icp_eigval))
          {
            int ord_bl = std_managers[id]->plane_cloud_vec_[search_result.first]->header.seq;

            IMUST &xx = multimap_scanPoses[id]->at(ord_bl)->x;
            double drift_p = (xx.R * loop_transform.first + xx.p - xc.p).norm();

            bool isPush = false;
            int step = -1;
            if(id == cur_id)
            {
              double span = smp->jour - keyframes->at(search_result.first)->jour;
              printf("drift: %lf %lf\n", drift_p, span);

              if(drift_p / span < ratio_drift)
              {
                isPush = true;
                step = stepsizes.size() - 2;

                if(relc_counts[id] > curr_halt && drift_p > 0.10)
                {
                  isOpt = true;
                  for(int &cnt: relc_counts) cnt = 0;
                }
              }
            }
            else
            {
              for(int i=0; i<ids.size(); i++)
                if(id == ids[i]) 
                  step = i;
              
              printf("drift: %lf %lf\n", drift_p, jours[id]);

              if(step == -1)
              {
                isGraph = true;
                isOpt = true;
                relc_counts[id] = 0;
                g_update = 1;
                isPush = true;
                jours[id] = 0;
              }
              else
              {
                if(drift_p / jours[id] < 0.05)
                {
                  jours[id] = 1e-6; // set to 0
                  isPush = true;
                  if(relc_counts[id] > prev_halt && drift_p > 0.25)
                  {
                    isOpt = true;
                    for(int &cnt: relc_counts) cnt = 0;
                  }
                }
              }

            }

            if(isPush)
            {
              match_num++;
              lp_edges.push(id, cur_id, ord_bl, buf_base-1, loop_transform.second, loop_transform.first, v6_init);
              if(step > -1)
              {
                int id1 = stepsizes[step] + ord_bl;
                int id2 = stepsizes.back() - 1;
                add_edge(id1, id2, loop_transform.second, loop_transform.first, graph, odom_noise);
                printf("addedge: (%d %d) (%d %d)\n", id, cur_id, ord_bl, buf_base-1);
              }
            }

            // if(isPush)
            // {
            //   icp_check(*(smp->plptr), *(std_managers[id]->plane_cloud_vec_[search_result.first]), pub_test, pub_init, loop_transform, multimap_scanPoses[id]->at(ord_bl)->x);
            // }

          }
        }
        
      }
      for(int &it: relc_counts) it++;
      std_manager->AddSTDescs(stds_vec);
  
      if(isGraph)
      {
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 1);
      }

      if(isOpt)
      {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);
        isam.update(graph, initial);

        for(int i=0; i<5; i++) isam.update();
        gtsam::Values results = isam.calculateEstimate();
        int resultsize = results.size();
        
        IMUST x1 = scanPoses->at(buf_base-1)->x;
        int idsize = ids.size();

        history_kfsize = 0;
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
          {
            int ord = j - stepsizes[ii];
            multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
          }
        }
        mtx_keyframe.lock();
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(Keyframe *kf: *multimap_keyframes[tip])
            kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
        }
        mtx_keyframe.unlock();

        initial.clear();
        for(int i=0; i<resultsize; i++)
          initial.insert(i, results.at(i).cast<gtsam::Pose3>());
        
        IMUST x3 = scanPoses->at(buf_base-1)->x;
        dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
        dx.R = x3.R * x1.R.transpose();
        x_key = x3;

        PVec pvec_tem;
        int subsize = keyframes->size();
        int init_num = 5;
        for(int i=subsize-init_num; i<subsize; i++)
        {
          if(i < 0) continue;
          Keyframe &sp = *(keyframes->at(i));
          sp.exist = 0;
          pvec_tem.reserve(sp.plptr->size());
          pointVar pv; pv.var.setZero();
          for(PointType &ap: sp.plptr->points)
          {
            pv.pnt << ap.x, ap.y, ap.z;
            pv.pnt = sp.x0.R * pv.pnt + sp.x0.p;
            for(int j=0; j<3; j++)
              pv.var(j, j) = ap.normal[j];
            pvec_tem.push_back(pv);
          }
          cut_voxel(map_loop, pvec_tem, win_size, 0);
        }

        if(subsize > init_num)
        {
          pl_kdmap->clear();
          for(int i=0; i<subsize-init_num; i++)
          {
            Keyframe &kf = *(keyframes->at(i));
            kf.exist = 1;
            PointType pp;
            pp.x = kf.x0.p[0]; pp.y = kf.x0.p[1]; pp.z = kf.x0.p[2];
            pp.intensity = cur_id; pp.curvature = i;
            pl_kdmap->push_back(pp);
          }

          kd_keyframes.setInputCloud(pl_kdmap);
          history_kfsize = pl_kdmap->size();
        }
        loop_detect = 1;

        vector<int> ids2 = ids; ids2.pop_back();
        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids2);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
        ids2.clear(); ids2.push_back(ids.back());
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);

      }

    }

    for(int i=0; i<std_managers.size(); i++)
      delete std_managers[i];
    malloc_trim(0);

    if(is_finish.load())
    {
      if(keyframes->empty())
      {
        sessionNames.pop_back();
        std_managers.pop_back();
        multimap_scanPoses.pop_back();
        multimap_keyframes.pop_back();
        juds.pop_back();
        jours.pop_back();
        relc_counts.pop_back();
      }

      if(multimap_keyframes.empty()) 
      {
        printf("no data\n"); return;
      }

      int cur_id = std_managers.size() - 1;
      build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 0);

      topDownProcess(initial, graph, ids, stepsizes);
    }

    if(is_finish.load() && operation_mode == "mapping" &&
       !map_export_path.empty() && !map_export_capture_failed)
    {
      string export_error;
      vector<CapturedPoseUpdate> pose_updates;
      unordered_set<const Keyframe*> unique_pose_sources;
      if(captured_map.keyframes.empty())
      {
        export_error = "no accepted fused keyframes were captured";
      }
      try
      {
        pose_updates.reserve(captured_pose_sources.size());
        unique_pose_sources.reserve(captured_pose_sources.size());
        for(const pair<int, const Keyframe*> &association:
            captured_pose_sources)
        {
          if(association.second == nullptr)
          {
            export_error = "final-pose association contains a null keyframe";
            break;
          }
          if(!unique_pose_sources.insert(association.second).second)
          {
            export_error =
                "final-pose association contains a duplicate keyframe";
            break;
          }
          CapturedPoseUpdate update;
          update.capture_id = association.first;
          update.pose.stamp = association.second->x0.t;
          update.pose.R = association.second->x0.R;
          update.pose.t = association.second->x0.p;
          pose_updates.push_back(update);
        }
      }
      catch(const std::bad_alloc&)
      {
        export_error = "out of memory while preparing final keyframe poses";
      }
      if(!export_error.empty() ||
         !refreshCapturedKeyframePoses(
             pose_updates, captured_map, export_error))
      {
        ROS_ERROR_STREAM("Map-package export aborted before save: "
                         << export_error);
      }
      else
      {
        MapPackageOptions export_options;
        export_options.win_size = win_size;
        export_options.keyframe_distance_m = 0.1;
        export_options.keyframe_angle_deg = 5.0;
        export_options.keyframe_voxel_size = voxel_size / 10.0;
        export_options.btc_config = config_setting;
        if(MapPackageIO::save(map_export_path, captured_map, export_options,
                              export_error))
        {
          ROS_INFO_STREAM("Saved optimized map package to "
                          << map_export_path << " with "
                          << captured_map.keyframes.size() << " keyframes.");
        }
        else
        {
          ROS_ERROR_STREAM("Map-package export failed: " << export_error);
        }
      }
    }

    if(is_save_map)
    {
      for(int i=0; i<ids.size(); i++)
        FileReaderWriter::instance().save_pose(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/alidarState.txt", savepath);

      FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 1, savepath, bagname);
    }

    for(int i=0; i<multimap_scanPoses.size(); i++)
    {
      for(int j=0; j<multimap_scanPoses[i]->size(); j++)
        delete multimap_scanPoses[i]->at(j);
    }
    for(int i=0; i<multimap_keyframes.size(); i++)
    {
      for(int j=0; j<multimap_keyframes[i]->size(); j++)
        delete multimap_keyframes[i]->at(j);
    }
    
    malloc_trim(0);
  }

  // The top down process of HBA
  void topDownProcess(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, vector<int> &ids, vector<int> &stepsizes)
  {
    cnct_map = ids;
    gba_size = multimap_keyframes.back()->size();
    gba_flag = 1;

    pcl::PointCloud<PointType> pl0;
    pub_pl_func(pl0, pub_pmap);
    pub_pl_func(pl0, pub_cmap);
    pub_pl_func(pl0, pub_curr_path);
    pub_pl_func(pl0, pub_prev_path);
    pub_pl_func(pl0, pub_scan);

    double t0 = ros::Time::now().toSec();
    while(gba_flag);
    
    for(PGO_Edge &edge: gba_edges1.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    for(PGO_Edge &edge: gba_edges2.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for(int i=0; i<5; i++) isam.update();
    gtsam::Values results = isam.calculateEstimate();
    int resultsize = results.size();

    int idsize = ids.size();
    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
      {
        int ord = j - stepsizes[ii];
        multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
      }
    }

    Eigen::Quaterniond qq(multimap_scanPoses[0]->at(0)->x.R);

    double t1 = ros::Time::now().toSec();
    printf("GBA opt: %lfs\n", t1 - t0);

    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(Keyframe *smp: *multimap_keyframes[tip])
        smp->x0 = multimap_scanPoses[tip]->at(smp->id)->x;
    }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
    vector<int> ids2 = ids; ids2.pop_back();
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
    ids2.clear(); ids2.push_back(ids.back());
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
  }

  // The bottom up to add edge in HBA
  void HBA_add_edge(vector<IMUST> &p_xs, vector<Keyframe*> &p_smps, PGO_Edges &gba_edges, vector<int> &maps, int max_iter, int thread_num, pcl::PointCloud<PointType>::Ptr plptr = nullptr)
  {
    bool is_display = false;
    if(plptr == nullptr) is_display = true;

    double t0 = ros::Time::now().toSec();
    vector<Keyframe*> smps;
    vector<IMUST> xs;
    int last_mp = -1, isCnct = 0;
    for(int i=0; i<p_smps.size(); i++)
    {
      Keyframe *smp = p_smps[i];
      if(smp->mp != last_mp)
      {
        isCnct = 0;
        for(int &m: maps)
        if(smp->mp == m)
        {
          isCnct = 1; break;
        }
        last_mp = smp->mp;
      }

      if(isCnct)
      {
        smps.push_back(smp);
        xs.push_back(p_xs[i]);
      }
    }
    
    int wdsize = smps.size();
    Eigen::MatrixXd hess;
    vector<double> gba_eigen_value_array_orig = gba_eigen_value_array;
    double gba_min_eigen_value_orig = gba_min_eigen_value;
    double gba_voxel_size_orig = gba_voxel_size;

    int up = 4;
    int converge_flag = 0;
    double converge_thre = 0.05;

    for(int iterCnt = 0; iterCnt < max_iter; iterCnt++)
    {
      if(converge_flag == 1 || iterCnt == max_iter-1)
      {
        // if(plptr == nullptr)
        // {
        //   break;
        // }

        gba_voxel_size = voxel_size;
        gba_eigen_value_array = plane_eigen_value_thre;
        gba_min_eigen_value = min_eigen_value;
      }

      unordered_map<VOXEL_LOC, OctreeGBA*> oct_map;
      for(int i=0; i<wdsize; i++)
        OctreeGBA::cut_voxel(oct_map, xs[i], smps[i]->plptr, i, wdsize);

      LidarFactor voxhess(wdsize);
      OctreeGBA_multi_recut(oct_map, voxhess, thread_num);

      Lidar_BA_Optimizer opt_lsv;
      opt_lsv.thd_num = thread_num;
      vector<double> resis;
      bool is_converge = opt_lsv.damping_iter(xs, voxhess, &hess, resis, up, is_display);
      if(is_display)
        printf("%lf\n", fabs(resis[0] - resis[1]) / resis[0]);
      if((fabs(resis[0] - resis[1]) / resis[0] < converge_thre && is_converge) || (iterCnt == max_iter-2 && converge_flag == 0))
      {
        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          converge_flag = 1;
        }
        else if(converge_flag == 1)
        {
          break;
        }
      }
    }

    gba_eigen_value_array = gba_eigen_value_array_orig;
    gba_min_eigen_value = gba_min_eigen_value_orig;
    gba_voxel_size = gba_voxel_size_orig;

    for(int i=0; i<wdsize - 1; i++)
    for(int j=i+1; j<wdsize; j++)
    {
      bool isAdd = true;
      Eigen::Matrix<double, 6, 1> v6;
      for(int k=0; k<6; k++)
      {
        double hc = fabs(hess(6*i+k, 6*j+k));
        if(hc < 1e-6) // 1e-6
        {
          isAdd = false; break;
        }
        v6[k] = 1.0 / hc;
      }

      if(isAdd)
      {
        Keyframe &s1 = *smps[i]; Keyframe &s2 = *smps[j];
        Eigen::Vector3d tra = xs[i].R.transpose() * (xs[j].p - xs[i].p);
        Eigen::Matrix3d rot = xs[i].R.transpose() *  xs[j].R;
        gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
      }
    }

    if(plptr != nullptr)
    {
      pcl::PointCloud<PointType> pl;
      IMUST xc = xs[0];
      for(int i=0; i<wdsize; i++)
      {
        Eigen::Vector3d dp = xc.R.transpose() * (xs[i].p - xc.p);
        Eigen::Matrix3d dR = xc.R.transpose() *  xs[i].R;
        for(PointType ap: smps[i]->plptr->points)
        {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = dR * v3 + dp;
          ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
          ap.intensity = smps[i]->mp;
          pl.push_back(ap);
        }
      }
      
      down_sampling_voxel(pl, voxel_size / 8);
      plptr->clear(); plptr->reserve(pl.size());
      for(PointType &ap: pl.points)
        plptr->push_back(ap);
    }
    else
    {
      // pcl::PointCloud<PointType> pl, path;
      // pub_pl_func(pl, pub_test);
      // for(int i=0; i<wdsize; i++)
      // {
      //   PointType pt;
      //   pt.x = xs[i].p[0]; pt.y = xs[i].p[1]; pt.z = xs[i].p[2];
      //   path.push_back(pt);
      //   for(int j=1; j<smps[i]->plptr->size(); j+=2)
      //   {
      //     PointType ap = smps[i]->plptr->points[j];
      //     Eigen::Vector3d v3(ap.x, ap.y, ap.z);
      //     v3 = xs[i].R * v3 + xs[i].p;
      //     ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      //     ap.intensity = smps[i]->mp;
      //     pl.push_back(ap);

      //     if(pl.size() > 1e7)
      //     {
      //       pub_pl_func(pl, pub_test);
      //       pl.clear();
      //       sleep(0.05);
      //     }
      //   }
      // }
      // pub_pl_func(pl, pub_test);
      // return;
    }

  }

  // The main thread of bottom up in global mapping
  void thd_globalmapping(ros::NodeHandle &n)
  {
    // Localization consumes an immutable historical map and never creates GBA
    // submaps. Do not start the mapping-only polling loop in this mode.
    if(operation_mode == "localization") return;
    n.param<double>("GBA/voxel_size", gba_voxel_size, 1.0);
    n.param<double>("GBA/min_eigen_value", gba_min_eigen_value, 0.01);
    n.param<vector<double>>("GBA/eigen_value_array", gba_eigen_value_array, vector<double>());
    for(double &iter: gba_eigen_value_array) iter = 1.0 / iter;
    int total_max_iter = 1;
    n.param<int>("GBA/total_max_iter", total_max_iter, 1);

    vector<Keyframe*> gba_submaps;
    deque<int> localID;

    int smp_mp = 0;
    int buf_base = 0;
    int wdsize = 10;
    int mgsize = 5;
    int thread_num = 5;

    while(n.ok())
    {
      if(multimap_keyframes.empty())
      {
        sleep(0.1); continue;
      }

      int smp_flag = 0;
      if(smp_mp+1 < multimap_keyframes.size() && !multimap_keyframes.back()->empty())
        smp_flag = 1;

      vector<Keyframe*> &smps = *multimap_keyframes[smp_mp];
      int total_ba = 0;
      if(gba_flag == 1 && smp_mp >= cnct_map.back() && gba_size <= buf_base)
      {
        printf("gba_flag enter: %d\n", gba_flag);
        total_ba = 1;
      }
      else if(smps.size() <= buf_base)
      {
        if(smp_flag == 0)
        {
          sleep(0.1); continue;
        }
      }
      else
      {
        smps[buf_base]->mp = smp_mp;
        localID.push_back(buf_base);

        buf_base++;
        if(localID.size() < wdsize)
        {
          sleep(0.1); continue;
        }
      }

      vector<IMUST> xs;
      vector<Keyframe*> smp_local;
      mtx_keyframe.lock();
      for(int i: localID)
      {
        xs.push_back(multimap_keyframes[smp_mp]->at(i)->x0);
        smp_local.push_back(multimap_keyframes[smp_mp]->at(i));
      }
      mtx_keyframe.unlock();

      double tg1 = ros::Time::now().toSec();

      Keyframe *gba_smp = new Keyframe(smp_local[0]->x0);
      vector<int> mps{smp_mp};
      HBA_add_edge(xs, smp_local, gba_edges1, mps, 1, 2, gba_smp->plptr);
      gba_smp->id = smp_local[0]->id;
      gba_smp->mp = smp_mp;
      gba_submaps.push_back(gba_smp);

      if(total_ba == 1)
      {
        printf("GBAsize: %d\n", gba_size);
        vector<IMUST> xs;
        mtx_keyframe.lock();
        for(Keyframe *smp: gba_submaps)
        {
          xs.push_back(multimap_scanPoses[smp->mp]->at(smp->id)->x);
        }
        mtx_keyframe.unlock();
        gba_edges2.edges.clear(); gba_edges2.mates.clear();
        HBA_add_edge(xs, gba_submaps, gba_edges2, cnct_map, total_max_iter, thread_num);

        if(is_finish.load())
        {
          for(int i=0; i<gba_submaps.size(); i++)
            delete gba_submaps[i];
        }
        gba_submaps.clear();

        malloc_trim(0);
        gba_flag = 0;
      }
      else if(smp_flag == 1 && multimap_keyframes[smp_mp]->size() <= buf_base)
      {
        smp_mp++; buf_base = 0; localID.clear();
        // printf("switch: %d\n", smp_mp);
      }
      else
      {
        for(int i=0; i<mgsize; i++)
          localID.pop_front();
      }
  
    }

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmn_voxel");
  ros::NodeHandle n;

  pub_cmap = n.advertise<sensor_msgs::PointCloud2>("/map_cmap", 100);
  pub_pmap = n.advertise<sensor_msgs::PointCloud2>("/map_pmap", 100);
  pub_scan = n.advertise<sensor_msgs::PointCloud2>("/map_scan", 100);
  pub_init = n.advertise<sensor_msgs::PointCloud2>("/map_init", 100);
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_curr_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_prev_path = n.advertise<sensor_msgs::PointCloud2>("/map_true", 100);
  
  VOXEL_SLAM vs(n);
  mp = new int[vs.win_size];
  for(int i=0; i<vs.win_size; i++)
    mp[i] = i;
  
  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(n));
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(n));
  vs.thd_odometry_localmapping(n);

  thread_loop.join();
  thread_gba.join();
  ros::spin(); return 0;
}

