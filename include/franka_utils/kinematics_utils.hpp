//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Michael Bombile (maintainer)
//|    email:   michael.bombile@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of iiwa_toolkit.
//|
//|    iiwa_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#ifndef __KinematicsUtils_H__
#define __KinematicsUtils_H__

#include "ros/ros.h"
#include "Eigen/Eigen"


template<typename T = float>
class KinematicsUtils 
{
	
	public:

	enum ROBOT_ID {KUKA_LWR, FRANKA_PANDA};


	enum DH_CONVENTION {NORMAL, MODIFIED};

	// Class constructor
	KinematicsUtils(){}

    static Eigen::Matrix<T,4,1> quaternionProduct(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2)
    {
	  Eigen::Matrix<T,4,1> q;
	  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
	  Eigen::Matrix<T,3,1> q1Im = (q1.segment(1,3));
	  Eigen::Matrix<T,3,1> q2Im = (q2.segment(1,3));
	  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

	  return q;
	}

    static Eigen::Matrix<T,3,3> getSkewSymmetricMatrix(Eigen::Matrix<T,3,1> input)
    {
	  Eigen::Matrix<T,3,3> output;

	  output << 0.0f, -input(2), input(1),
	            input(2), 0.0f, -input(0),
	            -input(1), input(0), 0.0f;

	  return output;
	}

	static Eigen::Matrix<T,3,3> eulerAnglesToRotationMatrix(T phi, T theta, T psi)
  	{
	  T cphi = std::cos(phi);
	  T sphi = std::sin(phi);
	  T ctheta = std::cos(theta);
	  T stheta = std::sin(theta);
	  T cpsi = std::cos(psi);
	  T spsi = std::sin(psi);


	  Eigen::Matrix<T,3,3> R;
	  R << cpsi*ctheta, cpsi*stheta*sphi-spsi*cphi, cpsi*stheta*cphi+spsi*sphi,
	       spsi*ctheta, spsi*stheta*sphi+cpsi*cphi, spsi*stheta*cphi-cpsi*sphi,
	       -stheta, ctheta*sphi, ctheta*cphi;

	  return R;
	}

    static Eigen::Matrix<T,4,1> rotationMatrixToQuaternion(Eigen::Matrix<T,3,3> R)
    {
	  Eigen::Matrix<T,4,1> q;

	  float r11 = R(0,0);
	  float r12 = R(0,1);
	  float r13 = R(0,2);
	  float r21 = R(1,0);
	  float r22 = R(1,1);
	  float r23 = R(1,2);
	  float r31 = R(2,0);
	  float r32 = R(2,1);
	  float r33 = R(2,2);

	  float tr = r11+r22+r33;
	  float tr1 = r11-r22-r33;
	  float tr2 = -r11+r22-r33;
	  float tr3 = -r11-r22+r33;

	  if(tr>0)
	  {  
	    q(0) = sqrt(1.0f+tr)/2.0f;
	    q(1) = (r32-r23)/(4.0f*q(0));
	    q(2) = (r13-r31)/(4.0f*q(0));
	    q(3) = (r21-r12)/(4.0f*q(0));
	  }
	  else if((tr1>tr2) && (tr1>tr3))
	  {
	    q(1) = sqrt(1.0f+tr1)/2.0f;
	    q(0) = (r32-r23)/(4.0f*q(1));
	    q(2) = (r21+r12)/(4.0f*q(1));
	    q(3) = (r31+r13)/(4.0f*q(1));
	  }     
	  else if((tr2>tr1) && (tr2>tr3))
	  {   
	    q(2) = sqrt(1.0f+tr2)/2.0f;
	    q(0) = (r13-r31)/(4.0f*q(2));
	    q(1) = (r21+r12)/(4.0f*q(2));
	    q(3) = (r32+r23)/(4.0f*q(2));
	  }
	  else
	  {
	    q(3) = sqrt(1.0f+tr3)/2.0f;
	    q(0) = (r21-r12)/(4.0f*q(3));
	    q(1) = (r31+r13)/(4.0f*q(3));
	    q(2) = (r32+r23)/(4.0f*q(3));        
	  }

	  return q;
	}

  	static Eigen::Matrix<T,3,3> quaternionToRotationMatrix(Eigen::Matrix<T,4,1> q)
  	{
	  Eigen::Matrix<T,3,3> R;

	  T q0 = q(0);
	  T q1 = q(1);
	  T q2 = q(2);
	  T q3 = q(3);

	  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
	  R(1,0) = 2.0f*(q1*q2+q0*q3);
	  R(2,0) = 2.0f*(q1*q3-q0*q2);

	  R(0,1) = 2.0f*(q1*q2-q0*q3);
	  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
	  R(2,1) = 2.0f*(q2*q3+q0*q1);

	  R(0,2) = 2.0f*(q1*q3+q0*q2);
	  R(1,2) = 2.0f*(q2*q3-q0*q1);
	  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

	  return R;
	}

	static void quaternionToAxisAngle(Eigen::Matrix<T,4,1> q, Eigen::Matrix<T,3,1> &axis, T &angle)
	{
	  if((q.segment(1,3)).norm() < 1e-3f)
	  {
	    axis = q.segment(1,3);
	  }
	  else
	  {
	    axis = q.segment(1,3)/(q.segment(1,3)).norm();
	    
	  }

	  angle = 2*std::acos(q(0));
	}

	static Eigen::Matrix<T,4,1> axisAngleToQuaterion(Eigen::Matrix<T,3,1> axis, T angle)
	{
	  Eigen::Matrix<T,4,1> q;
	  q(0) = std::cos(angle/2);
	  q(1) = axis(0)*std::sin(angle/2);
	  q(2) = axis(1)*std::sin(angle/2);
	  q(3) = axis(2)*std::sin(angle/2);
	  return q;
	}

  	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T t)
  	{

	  Eigen::Matrix<T,4,1> q;

	  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
	  if(q1.dot(q2)<0.0f)
	  {   
	    q2 = -q2;
	  }

	  T dotProduct = q1.dot(q2);
	  if(dotProduct > 1.0f)
	  {
	    dotProduct = 1.0f;
	  }
	  else if(dotProduct < -1.0f)
	  {
	    dotProduct = -1.0f;
	  }

	  T omega = acos(dotProduct);

	  if(std::fabs(omega)<FLT_EPSILON)
	  {
	    q = q1.transpose()+t*(q2-q1).transpose();
	  }
	  else
	  {
	    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
	  }

	  return q;
	}

	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1>* q, Eigen::Matrix<T,Eigen::Dynamic,1> t, int size)
	{

	  if(size==1)
	  {
	    return q[0];
	  }
	  else
	  {
	    T sum = 0.0f;
	    for(int k = 0; k <size; k++)
	    {
	      sum+=t(k);
	    }
	    if(sum<FLT_EPSILON)
	    {
	      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],0.0f);
	    }
	    else
	    {
	      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],t(size-1)/sum);
	    }
	  }
}

	static Eigen::Matrix<T,3,3> rodriguesRotation(Eigen::Matrix<T,3,1> v1, Eigen::Matrix<T,3,1> v2)
	{
	  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
	  v1.normalize();
	  v2.normalize();

	  Eigen::Matrix<T,3,1> w;
	  w = v1.cross(v2);
	  float c = v1.dot(v2);  
	  float s = w.norm();
	  w /= s;
	  
	  Eigen::Matrix<T,3,3> K;
	  K << getSkewSymmetricMatrix(w);

	  Eigen::Matrix<T,3,3> Re;
	  if(fabs(s)< FLT_EPSILON)
	  {
	    Re = Eigen::Matrix<T,3,3>::Identity();
	  }
	  else
	  {
	    Re = Eigen::Matrix<T,3,3>::Identity()+s*K+(1-c)*K*K;
	  }

	  return Re;
	}

	static Eigen::Matrix<T,3,1> quaternionToAngularVelocity(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T gain = 1.0f)
	{
	  Eigen::Matrix<T,4,1> q1I, wq;
	  q1I(0) = q1(0);
	  q1I.segment(1,3) = -q1.segment(1,3);
	  wq = 2.0f*gain*quaternionProduct(q2-q1,q1I);
	  
	  return wq.segment(1,3);
	}


	static Eigen::Matrix<T,3,3> orthogonalProjector(Eigen::Matrix<T,3,1> v)
	{ 
	  return Eigen::Matrix<T,3,3>::Identity()-v*v.transpose();
	}

  	static T smoothRise(T x, T a, T b)
  	{
	  T y; 
	  if(x<a)
	  {
	    y = 0.0f;
	  }
	  else if(x>b)
	  {
	    y = 1.0f;
	  }
	  else
	  {
	    y = (1.0f+sin(M_PI*(x-a)/(b-a)-M_PI/2.0f))/2.0f;
	  }

	  return y;
	}


	static T smoothFall(T x, T a, T b)
	{
	  return 1.0f-smoothRise(x,a,b);
	}


	static T smoothRiseFall(T x, T a, T b, T c, T d)
	{
	  return smoothRise(x,a,b)*smoothFall(x,c,d);
	}


	static T deadZone(T x, T a, T b)
	{
	  if(x < b && x > a)
	  {
	    return 0.0f;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,Eigen::Dynamic,1> deadZone(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
	{
	  T norm = x.norm();

	  if(norm>limit)
	  {
	    return x;
	  }
	  else
	  {
	    return Eigen::Matrix<T,Eigen::Dynamic,1>::Zero(x.size());
	  }
	}


	static T wrapToZero(T x, T a, T b)
	{
	  if(x < b && x > a)
	  {
	    return x;
	  }
	  else
	  {
	    return 0.0f;
	  }
	}


	static T bound(T x, T a, T b)
	{
	  if(x > b)
	  {
	    return b;
	  }
	  else if(x<a)
	  {
	    return a;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,Eigen::Dynamic,1> bound(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
	{
	  T norm = x.norm();

	  if(norm>limit)
	  {
	    return x*limit/norm;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,4,4> getDHMatrix(T a, T alpha, T d, T theta, DH_CONVENTION dhConvention = NORMAL)
	{
	  Eigen::Matrix<T,4,4> H;

	  if(dhConvention == NORMAL)
	  {
	    H(0,0) = std::cos(theta);
	    H(0,1) = -std::cos(alpha)*std::sin(theta);
	    H(0,2) = std::sin(alpha)*std::sin(theta);
	    H(0,3) = a*std::cos(theta);

	    H(1,0) = std::sin(theta);
	    H(1,1) = std::cos(alpha)*std::cos(theta);
	    H(1,2) = -std::sin(alpha)*std::cos(theta);
	    H(1,3) = a*std::sin(theta);

	    H(2,0) = 0.0f;
	    H(2,1) = std::sin(alpha);
	    H(2,2) = std::cos(alpha);
	    H(2,3) = d;

	    H(3,0) = 0.0f;
	    H(3,1) = 0.0f;
	    H(3,2) = 0.0f;
	    H(3,3) = 1.0f;
	  }
	  else
	  {
	    H(0,0) = std::cos(theta);
	    H(0,1) = -std::sin(theta);
	    H(0,2) = 0.0f;
	    H(0,3) = a;

	    H(1,0) = std::sin(theta)*std::cos(alpha);
	    H(1,1) = std::cos(theta)*std::cos(alpha);
	    H(1,2) = -std::sin(alpha);
	    H(1,3) = -d*std::sin(alpha);

	    H(2,0) = std::sin(theta)*std::sin(alpha);
	    H(2,1) = std::cos(theta)*std::sin(alpha);
	    H(2,2) = std::cos(alpha);
	    H(2,3) = d*std::cos(alpha);

	    H(3,0) = 0.0f;
	    H(3,1) = 0.0f;
	    H(3,2) = 0.0f;
	    H(3,3) = 1.0f;
	  }

	  return H;
	}

	static Eigen::Matrix<T,4,4> getForwardKinematics(Eigen::Matrix<T,7,1> joints, ROBOT_ID robotID = KUKA_LWR)
	{
	  Eigen::Matrix<T,4,4> H, H1, H2, H3, H4, H5, H6, H7, H8;

	  if(robotID==KUKA_LWR)
	  {
	    H1 = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
	    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
	    H3 = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
	    H4 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
	    H5 = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
	    H6 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
	    H7 = getDHMatrix(0.0f,0.0f,0.078f,joints(6));    
	    H = H1*H2*H3*H4*H5*H6*H7;
	  }
	  else if(robotID==FRANKA_PANDA)
	  {
	    H1 = getDHMatrix(0.0f,0.0f,0.333f,joints(0),MODIFIED);
	    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),MODIFIED);
	    H3 = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),MODIFIED);
	    H4 = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),MODIFIED);
	    H5 = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),MODIFIED);
	    H6 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),MODIFIED);
	    H7 = getDHMatrix(0.088f,M_PI/2.0f,0.0f,joints(6),MODIFIED);  
	    H8 = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
	    H = H1*H2*H3*H4*H5*H6*H7*H8;
	  }

	  return H;
	}


	static Eigen::Matrix<T,6,7> getGeometricJacobian(Eigen::Matrix<T,7,1> joints, Eigen::Matrix<T,3,1> rEEx = Eigen::Matrix<T,3,1>::Zero(), ROBOT_ID robotID = KUKA_LWR)
	{
	  Eigen::Matrix<T,4,4> Hee, H[8], Hk;
	  Eigen::Matrix<T,6,7> J;

	  DH_CONVENTION convention;
	  if(robotID == KUKA_LWR)
	  {
	    convention = NORMAL;
	  }
	  else if(robotID = FRANKA_PANDA)
	  {
	    convention = MODIFIED;
	  }

	  if(robotID == KUKA_LWR)
	  {  
	    H[0] = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
	    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
	    H[2] = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
	    H[3] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
	    H[4] = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
	    H[5] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
	    H[6] = getDHMatrix(0.0f,0.0f,0.078f,joints(6));
	    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
	  }
	  else if(robotID == FRANKA_PANDA)
	  {
	    H[0] = getDHMatrix(0.0f,0.0f,0.333f,joints(0),convention);
	    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),convention);
	    H[2] = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),convention);
	    H[3] = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),convention);
	    H[4] = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),convention);
	    H[5] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),convention);
	    H[6] = getDHMatrix(0.088f,M_PI/2.0f,0.107f,joints(6),convention);  
	    // H[7] = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
	    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
	  }

	  // std::cerr << H[0] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6] << std::endl << std::endl;

	  Eigen::Matrix<T,3,1> xEE, z0, x0, xk, zk;

	  xEE = Hee.block(0,3,3,1);



	  Hk.setIdentity();

	  J.setConstant(0.0f);



	  for(int k = 0; k < 7; k++)
	  {

	    if(convention==MODIFIED)
	    {
	      Hk = Hk*H[k];
	    }
	    xk = Hk.block(0,3,3,1);
	    zk = Hk.block(0,2,3,1);

	    J.block(0,k,3,1) = zk.cross(xEE-xk);
	    J.block(3,k,3,1) = zk;
	    if(convention == NORMAL)
	    {
	      Hk = Hk*H[k];
	    }
	  }

	  J.block(0,0,3,7) += -getSkewSymmetricMatrix(rEEx)*J.block(3,0,3,7); 

	  return J;

	}

	// //////////////////////////////////////////////////////////////////////////////
	static Eigen::Matrix<T,4,4> pose2HomoMx(Eigen::Matrix<T,3,1> x, Eigen::Matrix<T,4,1> q)
	{
	  Eigen::Matrix<T,4,4> H; H.setIdentity();
	  H.block(0,3,3,1) = x;
	  Eigen::Quaternion<T> q_(q(0), q(1), q(2), q(3));
	  H.block(0,0,3,3) = q_.toRotationMatrix();

	  return H;
	}

	static Eigen::Matrix<T,3,3> getCombinedRotationMatrix(T weight, Eigen::Matrix<T,3,3> w_R_c, Eigen::Matrix<T,3,3> w_R_d)
	{
	    Eigen::Quaternion<T> qc(w_R_c);                       // current 
	    Eigen::Quaternion<T> qd(w_R_d);                       // desired 
	    Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);
	    Eigen::Matrix<T,3,3> w_R_cd_t = q_t.toRotationMatrix();
	    
	    return q_t.toRotationMatrix();
	}


	static Eigen::Matrix<T,4,1> getSlerpInterpolation(T weight, Eigen::Matrix<T,3,3> w_R_c, Eigen::Matrix<T,3,3> w_R_d)
	{
	    Eigen::Quaternion<T> qc(w_R_c);                       // current 
	    Eigen::Quaternion<T> qd(w_R_d);                       // desired 
	    Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);

	    Eigen::Matrix<T,4,1> q_out;
	    q_out << q_t.w(), q_t.x(), q_t.y(), q_t.z();
	    
	    return q_out;
	}

	static Eigen::Matrix<T,6,1> getPoseErrorCur2Des(Eigen::Matrix<T,4,4> d_H_c)
	{   
	    // Pass 
	    Eigen::Matrix<T,6,1> d_eta_c(6);
	    d_eta_c.segment(0,3) << d_H_c(0,3), d_H_c(1,3), d_H_c(2,3);
	    // extracrion of the rotation
	    Eigen::Matrix<T,3,3> d_R_c = d_H_c.block(0,0,3,3);
	    Eigen::AngleAxis<T> d_AxisAngle_c(d_R_c);
	    Eigen::Matrix<T,3,1> d_Axis_c = d_AxisAngle_c.axis();
	    d_eta_c(3) = d_Axis_c(0) * d_AxisAngle_c.angle();
	    d_eta_c(4) = d_Axis_c(1) * d_AxisAngle_c.angle();
	    d_eta_c(5) = d_Axis_c(2) * d_AxisAngle_c.angle();

	    return d_eta_c;
	}

	static Eigen::Matrix<T,3,3> getMuThetaJacobian(Eigen::Matrix<T,3,3> d_R_c)
	{
	    // extracrion of the rotation
	    Eigen::AngleAxis<T> d_AxisAngle_c(d_R_c);
	    // function sinc(theta) and sinc(theta/2)
	    T sinc_theta, sinc_theta_2;
	    sinc_theta   = sin(d_AxisAngle_c.angle() + 1e-6)/(d_AxisAngle_c.angle() + 1e-6);
	    sinc_theta_2 = sin((d_AxisAngle_c.angle() + 1e-6)/2.)/((d_AxisAngle_c.angle() + 1e-6)/2.);
	    //
	    Eigen::Matrix<T,3,1> d_Axis_c = d_AxisAngle_c.axis();
	    Eigen::Matrix<T,3,3> Skew_Mu;    Skew_Mu.setZero(3,3);
	    //
	    Skew_Mu <<           0.0,   -d_Axis_c(2),    d_Axis_c(1),
	                 d_Axis_c(2),            0.0,   -d_Axis_c(0),
	                -d_Axis_c(1),    d_Axis_c(0),            0.0;

	    // Jacobian of the rotation
	    Eigen::Matrix<T,3,3> L_Mu_Theta;
	    L_Mu_Theta.setIdentity(3,3);
	    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c.angle()/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

	    return L_Mu_Theta;
	}

	static Eigen::Matrix<T,3,1> SaturationVect3(T lim, Eigen::Matrix<T,3,1> vel)
	{
	    Eigen::Matrix<T,3,1> v_ = vel;

	    if((fabs(vel(0))>lim) || (fabs(vel(1))>lim) || (fabs(vel(2))>lim)){
	        v_ = lim * (1./vel.norm() * vel);
	    }
	    return v_;
	}

	static Eigen::Matrix<T,6,1> SaturationTwist(T lim_l, T lim_a, Eigen::Matrix<T,6,1> vel)
	{
	    Eigen::Matrix<T,3,1> lin_, ang_;
	    lin_ = vel.head(3);
	    ang_ = vel.tail(3);

	    if((fabs(lin_(0))>lim_l) || (fabs(lin_(1))>lim_l) || (fabs(lin_(2))>lim_l)){
	        lin_ = lim_l * (1./lin_.norm() * lin_);
	    }
	    if((fabs(ang_(0))>lim_a) || (fabs(ang_(1))>lim_a) || (fabs(ang_(2))>lim_a)){
	        ang_ = lim_a * (1./ang_.norm() * ang_);
	    }

	    Eigen::Matrix<T,6,1> Vsat;
	    Vsat.head(3) = lin_;
	    Vsat.tail(3) = ang_;

	    return Vsat;
	}

	static Eigen::Matrix<T,12,12> getBimanualTaskTwistMapInv(T a_bi, T b_bi)
	{
	    // 
	    T a_bi_ = a_bi;
	    T b_bi_ = b_bi;
	    if(a_bi_!= 0.0)
	        b_bi_ = 1.0;
	    else if (a_bi_== 1.0)
	        b_bi_ = 0.0;

	    Eigen::Matrix<T, 12, 12> C_hands;
	    C_hands.setZero();
	    Eigen::Matrix<T, 6, 6> Idn;   Idn.setIdentity();
	    // Bimanual transformation
	    C_hands.topLeftCorner(6,6)     =               Idn;
	    C_hands.topRightCorner(6,6)    = -(1.-a_bi_) * Idn;
	    C_hands.bottomLeftCorner(6,6)  =      b_bi_  * Idn;
	    C_hands.bottomRightCorner(6,6) =       a_bi_ * Idn;
	    
	    return C_hands;
	}

	static void getBimanualTransforms(Eigen::Matrix<T,4,4> w_H_l, Eigen::Matrix<T,4,4> w_H_r, Eigen::Matrix<T,4,4> &w_H_a_, Eigen::Matrix<T,4,4> &l_H_r_)
	{
	  w_H_a_.setIdentity(4,4); 
	  l_H_r_.setIdentity(4,4); 
	  // relative transformation
	  // ========================
	  // l_H_r = W_H_l.inverse() * W_H_r;
	  l_H_r_.block(0,3, 3,1) = w_H_r.block(0,3, 3,1) - w_H_l.block(0,3, 3,1);       // translation expresse wrt. the world
	  l_H_r_.block(0,0, 3,3) = w_H_l.block(0,0, 3,3).transpose() * w_H_r.block(0,0, 3,3); // orienatation wrt. the left hand
	  // find the abolute transformation
	  // ======================================================
	  // Axis angle of relative hands orientation
	  Eigen::Matrix<T,3,3> l_R_r_ = l_H_r_.block(0,0, 3,3);
	  Eigen::AngleAxis<T> l_orientation_r(l_R_r_);
	  // Average orientation between hands
	  Eigen::Matrix<T,3,1> axis_ = l_orientation_r.axis();
	  T theta_  = 0.5*l_orientation_r.angle();
	  Eigen::AngleAxis<T> av_rot(theta_, axis_);

	  // Rotation matrix of the absolute hand frame expressed in the asbolute foot frame
	  Eigen::Matrix<T,3,3> W_R_a =  w_H_l.block(0,0, 3,3) * av_rot.toRotationMatrix();
	  w_H_a_.block(0,3, 3,1) = 0.5*(w_H_l.block(0,3, 3,1) + w_H_r.block(0,3, 3,1));

	  Eigen::Matrix<T,3,3> w_R_l = w_H_l.block(0,0, 3,3);
	  Eigen::Matrix<T,3,3> w_R_r = w_H_r.block(0,0, 3,3);
	  Eigen::Quaternion<T> qc(w_R_l);                       // current 
	  Eigen::Quaternion<T> qd(w_R_r);                       // desired 
	  T weight = 0.5;
	  Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);
	  Eigen::Matrix<T,3,3> w_R_cd_t = q_t.toRotationMatrix();
	    
	  w_H_a_.block(0,0, 3,3) = w_R_cd_t; //getCombinedRotationMatrix(weight, w_R_l, w_R_r);

	}

	static void getBimanualTwistDistribution(T a_bi, T b_bi, Eigen::Matrix<T,6,1> vel_a, Eigen::Matrix<T,6,1> vel_r, 
                                            Eigen::Matrix<T,6,1> &left_V, Eigen::Matrix<T,6,1> &right_V)
	{
	  Eigen::Matrix<T,12,12> Th = KinematicsUtils<T>::getBimanualTaskTwistMapInv(a_bi, b_bi);
	  //
	  left_V  = Th.topLeftCorner(6,6)  *vel_a
	          + Th.topRightCorner(6,6) *vel_r;
	  //
	  right_V = Th.bottomLeftCorner(6,6)  *vel_a
	          + Th.bottomRightCorner(6,6) *vel_r;
	}

	static Eigen::Matrix<T,3,1> getEulerAnglesXYZ_FixedFrame(Eigen::Matrix<T,3,3> R)
  {
      // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
      // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
      Eigen::Matrix<T,3,1> Angles;
      T Psi_X, Theta_Y, Phi_Z;
          Psi_X   = std::atan2(R(2,1),R(2,2));
          Theta_Y = std::atan2(-R(2,0), fabs(std::sqrt(std::pow(R(0,0), 2.)+std::pow(R(1,0), 2.))));
          Phi_Z   = std::atan2(R(1,0),R(0,0));
      if ((Theta_Y>M_PI/2.)||(Theta_Y<-M_PI/2.))
      {
          Psi_X   = std::atan2(-R(2,1),-R(2,2));
          Theta_Y = std::atan2(-R(2,0),-fabs(std::sqrt(std::pow(R(0,0), 2.)+std::pow(R(1,0), 2.))));
          Phi_Z   = std::atan2(-R(1,0),-R(0,0));
      }
      Angles(0) = Psi_X;
      Angles(1) = Theta_Y;
      Angles(2) = Phi_Z;

      return Angles;
  }

};

template class KinematicsUtils<float>;
template class KinematicsUtils<double>;

#endif