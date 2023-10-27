/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           3D-vector MFP updater implementation
* \ingroup         PF
* \file            UpdateMFP3D.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef UPDATE_MODEL_MFP_3D_H
#define UPDATE_MODEL_MFP_3D_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include "../magnetic/MFP.h"
#include <cmath>
#include <eigen/Geometry>

namespace PF
{
	/**
	* 3D vector magnetic updater implementation
	*
	* uses predicted meaasurements from RBF part as predictions and Mag-map for updates
	*/
	class UpdateMFP3D : public UpdateSource
	{
	public:
		UpdateMFP3D()
		{
			initialize(0);
		}
		UpdateMFP3D(IFFData<> *pIFFData)
		{
			initialize(pIFFData);
		}

		virtual ~UpdateMFP3D()
		{
			delete[] pMfpParticles3D;
			delete[] pMfpWeights;
		};

		void initialize(IFFData<> *pIFFData)
		{
			isEnabled = false;
			pIFF = pIFFData;
			pMfpParticles3D = 0;
			pMfpWeights = 0;
			mfpSize = 0;
			b_x = b_y = b_z = 0;
			bias = false;
			name = "UpdateMFP3D";
			mfp_map_matching_enabled = false;
			prev_x = 0;
			prev_y = 0;
		}


		virtual bool enabled()
		{
			return isEnabled;
		}
		void  setUpdate(bool flag)
		{
			isEnabled = flag;
		}

		void setName(const std::string &objName)
		{
			name = objName;
		}

		void set_mfp_map_matching(bool is_enabled)
		{
			mfp_map_matching_enabled = is_enabled;
		}


		void print(std::ostream  &os) const
		{
			bool attitude(false), is_motion(false);
			double m_x(0), m_y(0), m_z(0);
			void * ptr(0);

			if (pIFF != 0)
			{
				ptr = pIFF->getMFP();
				attitude = pIFF->getPrediction().quat_valid;
				is_motion = pIFF->getPrediction().is_motion;
				m_x = pIFF->getMagMeas().m(0);
				m_y = pIFF->getMagMeas().m(1);
				m_z = pIFF->getMagMeas().m(2);
			}


			os << name << "<UpdMfp3D>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << mfpSize << " "
				<< attitude << " " << is_motion << " "
				<< m_x << " " << m_y << " " << m_z << " "
				<< b_x << " " << b_y << " " << b_z << std::endl;

		}

	protected:
		/**
		* update method
		*/
		virtual void update(IFilter *pFilter)
		{
			bool attitude(false), is_motion(true), has_map(false);
			double cur_x = 0;
			double cur_y = 0;
			double distance;

			if (pIFF != 0)
			{
				has_map = (pIFF->getMFP() != 0);
				attitude = pIFF->getPrediction().quat_valid;
				is_motion = pIFF->getPrediction().is_motion;
				cur_x = pIFF->getPrediction().x;
				cur_y = pIFF->getPrediction().y;
				distance = sqrt((cur_x - prev_x)*(cur_x - prev_x) + (cur_y - prev_y)*(cur_y - prev_y));
				prev_x = cur_x;
				prev_y = cur_y;
			}

			if (has_map && enabled() /*&& attitude && is_motion*/) // SHOULD BE REWIEVED
			{
				int N = pFilter->getParcticlesCount();
				resizeMfpArrays(N);
				Particle *state_est = pFilter->getState();
				Particle *state_prop = pFilter->getPrediction();

				Eigen::Matrix<Float, 3, 3> Ci2l = pIFF->getPrediction().qi2l.toRotationMatrix();
				Eigen::Matrix<Float, 3, 3> Rz = Eigen::Matrix<Float, 3, 3>::Zero();
				Eigen::Matrix<Float, 3, 3> C;
				Eigen::Matrix<Float, 3, 1> v, m = pIFF->getMagMeas().m;

				Rz(2, 2) = 1;

				for (int i = 0; i < N; ++i)
				{
					double h = state_prop[i].h;

					Rz(0, 0) = cos(h); Rz(0, 1) = -sin(h);
					Rz(1, 0) = -Rz(0, 1); Rz(1, 1) = Rz(0, 0);

					C = Rz * Ci2l;
					v = m - state_prop[i].rbf.b;
					v = C * v;
					//v = Ci2l*Rz*v;

					pMfpParticles3D[i].v[0] = v(0);
					pMfpParticles3D[i].v[1] = v(1);
					pMfpParticles3D[i].v[2] = v(2);

					//pMfpParticles3D[i].s[0] = sqrt(state_prop[i].rbf.S(0, 0));
					//pMfpParticles3D[i].s[1] = sqrt(state_prop[i].rbf.S(1, 1));
					//pMfpParticles3D[i].s[2] = sqrt(state_prop[i].rbf.S(2, 2));

					auto S = C * state_prop[i].rbf.S * C.transpose();
					pMfpParticles3D[i].s[0] = sqrt(S(0, 0));
					pMfpParticles3D[i].s[1] = sqrt(S(1, 1));
					pMfpParticles3D[i].s[2] = sqrt(S(2, 2));

					pMfpParticles3D[i].X = state_prop[i].pos.x * 100;
					pMfpParticles3D[i].Y = state_prop[i].pos.y * 100;
					pMfpParticles3D[i].Floor = (int)floor(state_prop[i].pos.level + 0.5);
				}

				(pIFF->getMFP())->CheckParticles3D(pMfpParticles3D, pMfpWeights, N, mfp_map_matching_enabled);

				MotionModelParams* pMotion_params = pFilter->GetMotionParams();
				VenueType venue_type = pFilter->getVenueType();

				for (int i = 0; i < N; ++i)
				{
					auto d = m - state_prop[i].rbf.z; // d = B - z
                        
					if (!state_prop[i].lkh)
					{
						if (attitude && is_motion && !state_prop[i].portal_type)
						{
                            double w = calcParticleWeigth(d, state_prop[i].rbf.S, pFilter->is_in_tracking(), venue_type);
							state_est[i].w = (pMfpWeights[i] > 0) ? (state_est[i].w * w) : 0;
							double a = 1. / 20;
							state_prop[i].rbf.C = (1 - a) * (state_prop[i].rbf.C + a * (d * d.transpose()));
						}
					}
					else
					{
                        double w = calcParticleWeigthFromMotionModel(state_prop[i], state_est[i], pMotion_params);
						state_est[i].w = state_est[i].w * w;
					}
				}
				double sumLk(0), sumNoLk(0), sumNoUpd(0);
				int numLkh(0), numNoLkh(0), numNoUpd(0);
				for (int i = 0; i < N; i++)
				{
					if (state_prop[i].portal_type)
					{
						sumNoUpd += state_est[i].w;
						numNoUpd++;
					}
					else
					{
						if (!state_prop[i].lkh)
						{
							sumNoLk += state_est[i].w;
							numNoLkh++;
						}
						else if (state_prop[i].lkh)
						{
							sumLk += state_est[i].w;
							numLkh++;
						}
						else
						{
							sumNoUpd += state_est[i].w;
							numNoUpd++;
						}
					}
				}

                    //if (numLkh > 0) disabled as unnecesarily condition causing to bag with portal particles
				{
					double gamma = ((double)numNoLkh) / N, alpha = ((double)numLkh) / N;
					double beta = ((double)numNoUpd) / N;
					if (numLkh > 0)
						sumLk /= alpha;

					if (numNoLkh > 0)
						sumNoLk /= gamma;

					if (numNoUpd > 0)
						sumNoUpd /= beta;

					for (int i = 0; i < N; i++)
					{
						if (state_prop[i].portal_type)
						{
							if (numNoUpd > 0)
								//state_est[i].w = 1. / (double)N;
								state_est[i].w /= sumNoUpd;
						}
						else
						{
							if (!state_prop[i].lkh)
							{
								if (sumNoLk > 0)
								{
									state_est[i].w /= sumNoLk;
								}
							}
							else if (state_prop[i].lkh)
							{
								if (sumLk > 0)
								{
									state_est[i].w /= sumLk;
								}
							}
							else
							{
								if (numNoUpd > 0)
									//state_est[i].w = 1. / (double)N;
									state_est[i].w /= sumNoUpd;
							}
						}
					}
#if 0 // check weights sum
                    static int mfp_update_count = 0;
                    mfp_update_count++;

                    double sum = 0;
                    double floor2 = 0;
                    for (int i = 0; i < N; ++i)
                    {
                        sum += state_est[i].w;
                        floor2 += state_est[i].w * state_est[i].pos.level;
                    }
                    sumLk = 0;
                    sumNoLk = 0;
                    sumNoUpd = 0; 
                    for (int i = 0; i < N; i++)
                    {
                        if (state_prop[i].portal_type)
                            sumNoUpd += state_est[i].w;
                        else
                        {
                            if (!state_prop[i].lkh)
                                sumNoLk += state_est[i].w;
                            else if (state_prop[i].lkh)
                                sumLk += state_est[i].w;
                            else
                                sumNoUpd += state_est[i].w;
                        }
                    }
                    {
                        std::cout << "mfp_update: ,       " << mfp_update_count << "   ,       "
                            << "NoUpd:  ,      " << numNoUpd << "     ,     " << sumNoUpd << "     ,       "
                            << "NoLkh:  ,      " << numNoLkh << "   ,     " << sumNoLk << "      ,      "
                            << "Lkh:   ,     " << numLkh << "   ,   " << sumLk << "   ,   "
                            << std::endl;
                    }
#endif
				}
			}
		}

		virtual PF::Float calcParticleWeigthFromMotionModel(Particle state, Particle est, MotionModelParams* pMotion_params)
		{
			double sig_x, sig_y, sig_h, sig_f, w;
			double dx, dy, dh, df;
			double Fxy, Fh, Fxyh, Ff;
			sig_h = pMotion_params->sig_h;
			sig_x = pMotion_params->sig_x;
			sig_y = pMotion_params->sig_y;
			sig_f = pMotion_params->sig_f;
			dx = state.pos.x - est.pos.x;
			dy = state.pos.y - est.pos.y;
			df = state.pos.level - est.pos.level;
			double head = state.h;
			dh = head - est.h;
			if (dh > M_PI)
				dh -= M_PI * 2;
			else if (dh < -M_PI)
				dh += M_PI * 2;

			Fxyh = 1. / (2 * M_PI*sig_x*sig_y)*exp(-dx * dx / (2 * sig_x*sig_x) - dy * dy / (2 * sig_y*sig_y));
			w = std::max(Fxyh, std::numeric_limits<double>::min());
			w = pow(w, 1. / 4);
			return w;
		}

		virtual PF::Float calcParticleWeigth(Eigen::Matrix<PF::Float, 3, 1> d, Eigen::Matrix<PF::Float, 3, 3> S, bool is_in_tracking, VenueType venue_type)
		{
			PF::Float detS = S.determinant();
			Eigen::Matrix<PF::Float, 3, 3> invS = S.inverse();

			double e = (d).transpose() * invS * (d);
			// Coef = gamma((v+p)/2)/(gamma(v/2)*v^p/2*pi^p/2)
			//double p = 3, v = 0.31, T= 2.0030, Coef = 0.649212407853154;
			//double p = 3, v = 0.5, T = 0.99, Coef = 0.261616025827484;
			//double p = 3, v = 0.372, T = 1.5040, Coef = 0.458054854029500;
			//double p = 3, v = 0.275, T = 2.8420, Coef = 0.817446233039943;
			double p = 3, v = 5, T = 0, Coef = 0.004658170823845;
			//double p = 3, v = 3, T = 0, Coef = 0.010782802832974;
			//double p = 3, v = 10, T = 0, Coef = 0.001547456991711;

			double w;
			if (venue_type == kAisleVenue)
			{
				if (e >= T)//Student
					w = Coef / sqrt(detS) * pow((1. + e / v), -(v + p) / 2);
				else//Gauss
					w = 1. / (pow(2 * M_PI, 3. / 2) * sqrt(detS)) * exp(-e / 2.);
				w = is_in_tracking ? pow(w, 1. / 2) : w;
			}
			else
			{
				e = (e > 2.) ? (log(8. * e) / log(4.)) : e;
				w = 1. / (pow(2 * M_PI, 3. / 2) * sqrt(detS)) * exp(-e / 2.);
				w = is_in_tracking ? pow(w, 1. / 3) : w;
			}
			return w;
		}

		void resizeMfpArrays(int new_size)
		{
			if (new_size != mfpSize)
			{
				delete[] pMfpParticles3D;
				delete[] pMfpWeights;

				pMfpParticles3D = new tMFP_Particle3D[new_size];
				pMfpWeights = new double[new_size];
				mfpSize = new_size;
			}
		}

		IFFData<> *pIFF;
		bool  isEnabled;
		tMFP_Particle3D *pMfpParticles3D;
		double *pMfpWeights;
		int mfpSize;
		Float b_x, b_y, b_z;
		bool bias;
		bool mfp_map_matching_enabled;
		std::string name;
		double prev_x;
		double prev_y;
	};

	class UpdateMFP3D_Init : public UpdateMFP3D
	{
	public:
		void initialize(IFFData<> *pIFFData)
		{
			isEnabled = false;
			pIFF = pIFFData;
			pMfpParticles3D = 0;
			pMfpWeights = 0;
			mfpSize = 0;
			b_x = b_y = b_z = 0;
			bias = false;
			name = "UpdateMFP3D_Init";
			mfp_map_matching_enabled = false;
		}
	private:
		virtual PF::Float calcParticleWeigth(Eigen::Matrix<PF::Float, 3, 1> d, Eigen::Matrix<PF::Float, 3, 3> S, bool is_in_tracking, VenueType venue_type)
		{
			PF::Float detS = S.determinant();
			Eigen::Matrix<PF::Float, 3, 3> invS = S.inverse();

			double e = (d).transpose() * invS * (d);
			// Coef = gamma((v+p)/2)/(gamma(v/2)*v^p/2*pi^p/2)
			//double p = 3, v = 0.31, T= 2.0030, Coef = 0.649212407853154;
			//double p = 3, v = 0.5, T = 0.99, Coef = 0.261616025827484;
			//double p = 3, v = 0.372, T = 1.5040, Coef = 0.458054854029500;
			//double p = 3, v = 0.275, T = 2.8420, Coef = 0.817446233039943;
			double p = 3, v = 5, T = 0, Coef = 0.004658170823845;
			//double p = 3, v = 3, T = 0, Coef = 0.010782802832974;
			//double p = 3, v = 10, T = 0, Coef = 0.001547456991711;
			double w;
			if (venue_type == kAisleVenue)
			{
				if (e >= T)//Student
					w = Coef / sqrt(detS) * pow((1. + e / v), -(v + p) / 2);
				else // Gauss
					w = 1. / (pow(2 * M_PI, 3. / 2) * sqrt(detS)) * exp(-e / 2.);
			}
			else
			{
				e = (e > 2.) ? (log(8. * e) / log(4.)) : e;
				w = 1. / (pow(2 * M_PI, 3. / 2) * sqrt(detS)) * exp(-e / 2.);
			}

			return w;
		}

		virtual PF::Float calcParticleWeigthFromMotionModel(Particle state, Particle est, MotionModelParams* pMotion_params)
		{
			double sig_x, sig_y, sig_h, sig_f;
			double dx, dy, dh, df;
			double Fxy, Fh, Fxyh, Ff;
			sig_h = pMotion_params->sig_h; //3.*M_PI / 180.;
			sig_x = pMotion_params->sig_x;
			sig_y = pMotion_params->sig_y;
			sig_f = pMotion_params->sig_f;
			dx = state.pos.x - est.pos.x;
			dy = state.pos.y - est.pos.y;
			df = state.pos.level - est.pos.level;
			double head = state.h;
			dh = head - est.h;
			if (dh > M_PI)
				dh -= M_PI * 2;
			else if (dh < -M_PI)
				dh += M_PI * 2;

			Fxyh = 1. / (2 * M_PI*sig_x*sig_y)*exp(-dx * dx / (2 * sig_x*sig_x) - dy * dy / (2 * sig_y*sig_y));
			return std::max(Fxyh, std::numeric_limits<double>::min());
		}
	};
}

#endif
