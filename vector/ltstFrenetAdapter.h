/**	\ingroup	ltstFrenetAdapter
\{ */

/**
 *
 * FrenetSpaceLib class declaration.
 *
 * @file
 * Copyright &copy; Audi AG. All rights reserved.
 *
 * $LastChangedBy$
 * $Date$
 * $Revision$
 *
 * @remarks
 *
 */

#pragma once
#include <algorithm>

#include <frenetSpace/frenetspace.h>
 //#include <latStabPlanner/ltstReference.h>


namespace targetpoint
{
	constexpr int SAMPLELENGTH = 4; // prefer to use constexpr instead of #define

	typedef tFloat32 tSampleFloat;

	struct tSample
	{
		typedef tSampleFloat value_type;
		typedef tUInt8 size_type;
		static const size_type m_nMaxSize = SAMPLELENGTH;
		tSampleFloat afData[SAMPLELENGTH]; ///< Sample with ::SAMPLELENGTH derivations \f$[m], [\frac{m}{s}], [\frac{m}{s^2}] ...\f$
		tSampleFloat fTime;                ///[s] time of the sample
		static size_type max_size() { return m_nMaxSize; }
		PADDING32
	};

}




namespace frenetspace
{
	enum eRefPathType
	{
		FR_PATHTYPE_UNDEFINED = 0,
		FR_PATHTYPE_PATH,         ///< reference path is TL_tPath see TrajectoryLib
		FR_PATHTYPE_PATHEXT,      ///< reference path is TL_tPathExt see TrajectoryLib
		FR_PATHTYPE_PATHEXT_REL,  ///< reference path is TL_tPathExtRel see TrajectoryLib
		FR_PATHTYPE_TRAJECTORY,   ///< reference path is TL_tTrajectory see TrajectoryLib
		FR_PATHTYPE_NO_OF
	};

	typedef frenetspacelib::tSample<tSampleFloat, frenetspacelib::SAMPLE_J> tSample;

	typedef tFloat64 tEgoDynFloat;
	typedef tFloat32 tEgoShapeFloat;

	/// \brief Ego delta motion in frenet space
/// \details Contains rotation and translation from current to last planning step
	struct tDeltaEgoMotionFrenet
	{
		tEgoDynFloat fdS;       ///<\f$[m]\f$   \f$\Delta s\f$ Movement in longitudinal direction
		tEgoDynFloat fdD;       ///<\f$[m]\f$   \f$\Delta d\f$ Movement in lateral direction
		tEgoDynFloat fdPsi;     ///<\f$[rad]\f$ \f$\Delta \psi\f$ Movement angle
	};

	///\brief Ego shape model
/// \details 
/// \code
///             fX0VehicleFront
///     +----+---------------------+
///     |    |                     |
///     |    |                     |
///     |    |                     |   fVehicleWidth
///     |    |                     |
///     |    |                     |
///     +--------------------------+
///           fVehicleLength
/// \endcode
///
	struct tShapeModel
	{
		tEgoShapeFloat fVehicleWidth;         ///<\f$[m]\f$ width of vehicle 
		tEgoShapeFloat fVehicleLength;        ///<\f$[m]\f$ length of vehicle
		tEgoShapeFloat fX0VehicleFront;       ///<\f$[m]\f$ length of vehicle from USK Origin
		PADDING32;
	};

	///\struct  tEgoDynamicsFrenet
	///
	///\brief   Ego dynamic.
	///\details  Contains acceleration \f$a\f$, velocity \f$v\f$ for both dimensions and yaw rate
	///                \f$\psi\f$.
	///\author  U 2ary 9w
	///\date    21.10.2016
	struct tEgoDynamicsFrenet
	{
		targetpoint::tSample          sLatDyn;  ///< dynamic in frenet space in lateral direction 
		targetpoint::tSample          sLongDyn; ///< dynamic in frenet space in longitudinal direction
		tEgoDynFloat                  fYawRate; ///<\f$[rad]\f$   \f$\psi\f$ yaw rate
		//PADDING32
	};

	struct tEgoFrenet
	{
		tShapeModel             sShape;
		tDeltaEgoMotionFrenet   sDeltaMotion;
		tDeltaEgoMotionFrenet   sDeltaMotionPlanning;
		tEgoDynamicsFrenet      sEgoDynFrenet;
	};



	/**
	 * \brief Contains functions to transform USK-positions / -dynamics to frenet space and back
	 *
	 * <a href="\\\\\AUDIINTEPro.in.audi.vwg\ProjectS\EX_Automatisiertes_Fahren\EX_VE_Konzepte\B000_AI_LAB\B400_Querschnittsfunktionen\B420_Funktion\04_Planer\AI_LAB_Verfahren\FrenetRaum.docx">FrenetSpace.docx</a>
	 */
	class cFrenetSpace
	{

	public:

		/**
		 *  structure containing all information to transform longitudinal samples with all lateral samples
		 */
		template<int noOfPoints>
		struct tCalculationData
		{
			frenetspacelib::tFRTvsUSK< fSpaceFloat> asFRTvsUSK[noOfPoints]; ///< conversion information for each point
			//configspace::space::tFRTvsUSK asFRTvsUSK[noOfPoints];   
			tUInt16 nLastIdx;                   ///< last closest index
			tBool bCalculated;                  ///< all components for current longitunal trajectory calculated
			//static const tUInt16 NOOFPOINTSMAX; 
		};

	private:
		frenetspacelib::tFrenetRefPath m_sRefPath;
		frenetspacelib::tFRTvsUSK< fSpaceFloat> m_sStartFRTUSK;

		TL_tTrajPoint m_sUSKStartPoint;
		TL_tTrajPoint m_sUSKEndPoint;

		targetpoint::tSample m_sLongCur, m_sLatCur;
		targetpoint::tSample m_sLongLast, m_sLatLast;

		tFloat32 m_fDebugInfoFRatio;
		tUInt16 m_nDebugInfoNIdx;
		tBool m_bDebugInfoDo;

		TL_tTrajPoint m_sUSKRef;

		TL_tPathExt m_PathExt;

	public:

		cFrenetSpace()
			: m_fDebugInfoFRatio(0)
			, m_nDebugInfoNIdx(0)
			, m_bDebugInfoDo(tFalse)
		{};

		/** @brief get lengthoffset to ego position
		 *
		 *  @return  the distance of the first reference point to ego position
		 */
		tFrenetCalcFloat getStartLength() { return m_sRefPath.aPoint[0].fL; };


		/**
		 *  @brief translate start dynamic from USK to frenet space
		 *
		 *  @param i_sTrajPoint point in USK
		 *  @param o_sSampleS   longitudinal dynamic data in frenet space
		 *  @param o_sSampleD   lateral dynamic data in frenet space
		 */
		tVoid GetStartDyn(IN const	TL_tTrajPoint			&i_sTrajPoint,
			OUT		targetpoint::tSample	&o_sSampleS,
			OUT		targetpoint::tSample	&o_sSampleD)
		{
			frenetspacelib::USK2FRT(i_sTrajPoint, m_sRefPath, o_sSampleS, o_sSampleD, m_sStartFRTUSK);

			/// catch standstill case and negative velocities
			if (o_sSampleS.afData[frenetspacelib::SAMPLE_V] < 0 && o_sSampleS.afData[frenetspacelib::SAMPLE_V] > -0.1f)
			{
				o_sSampleS.afData[frenetspacelib::SAMPLE_V] = 0;
				o_sSampleD.afData[frenetspacelib::SAMPLE_V] = 0;
			}
		};

		tBool calculateDelta(IN	const TL_tTrajPoint		&i_sPointLast,
			IN const TL_tTrajPoint		&i_sPointCur,
			OUT tDeltaEgoMotionFrenet	&o_sDeltaMotion)
		{

			GetStartDyn(i_sPointLast, m_sLongLast, m_sLatLast);

			GetStartDyn(i_sPointCur, m_sLongCur, m_sLatCur);

			//Calculate Deltas
			//Position 
			o_sDeltaMotion.fdD = m_sLatCur.afData[frenetspacelib::SAMPLE_POS] - m_sLatLast.afData[frenetspacelib::SAMPLE_POS];
			o_sDeltaMotion.fdS = m_sLongCur.afData[frenetspacelib::SAMPLE_POS] - m_sLongLast.afData[frenetspacelib::SAMPLE_POS];

			//Angle
			frenetadapter::GeoTools::GeoTrafo::cVector2D sLast;
			sLast.fX = m_sLongLast.afData[frenetspacelib::SAMPLE_V];
			sLast.fY = m_sLatLast.afData[frenetspacelib::SAMPLE_V];
			frenetadapter::GeoTools::GeoTrafo::cVector2D sCur;
			sCur.fX = m_sLongCur.afData[frenetspacelib::SAMPLE_V];
			sCur.fY = m_sLatCur.afData[frenetspacelib::SAMPLE_V];
			o_sDeltaMotion.fdPsi = frenetadapter::GeoTools::Signed_Angle_2D(sLast, sCur);
			return 0;

		};

		tBool transformStartDyn(IN const lmState_T			&advanceState,
			IN const curveAdvance_T	&curveAdvance,
			IN const real32_T			&initialVelocity,
			IN const real32_T			&initialAcceleration,
			IN const real32_T			&initialCurvature,
			IN const real32_T			time)
		{
			m_sUSKStartPoint.SetX(advanceState.posX);
			m_sUSKStartPoint.SetY(advanceState.posY);
			m_sUSKStartPoint.SetTangentialAngle(advanceState.yawAngle);
			m_sUSKStartPoint.SetCurvature(curveAdvance.curvature);
			m_sUSKStartPoint.fV = initialVelocity;
			m_sUSKStartPoint.fAx = initialAcceleration;
			//m_sUSKStartPoint.fC0 = initialCurvature;
			m_sUSKStartPoint.fAy = m_sUSKStartPoint.fV*m_sUSKStartPoint.fV*m_sUSKStartPoint.fC0;
			m_sUSKStartPoint.fTime = time;

			GetStartDyn(m_sUSKStartPoint, m_sLongCur, m_sLatCur);

			return tTrue;
		};
		// Get the data for the end condition from the reference trajectory and check if the latReference end data contains non-zero PosX
		tBool transformEndDyn(IN const ltstReference_T &latReference)
		{
			int endIndex = 0;

			for (int endIndex = latReference.count; endIndex > 0; endIndex--)
			{
				if (latReference.nodes[endIndex].constraints.posX > 0)
				{
					break;
				}

			}
			m_sUSKEndPoint.SetX(latReference.nodes.at(endIndex).constraints.posX);
			m_sUSKEndPoint.SetY(latReference.nodes.at(endIndex).constraints.centerY);
			m_sUSKEndPoint.SetTangentialAngle(latReference.nodes.at(endIndex).constraints.centerYaw);
			m_sUSKEndPoint.SetCurvature(latReference.nodes.at(endIndex).constraints.centerC0);
			m_sUSKEndPoint.fV = latReference.nodes.at(endIndex).constraints.velocity; //nodes[endIndex].velocity;
			m_sUSKEndPoint.fAx = latReference.nodes.at(endIndex).constraints.acceleration;
			m_sUSKEndPoint.fAy = m_sUSKEndPoint.fV*m_sUSKEndPoint.fV*m_sUSKEndPoint.fC0;
			m_sUSKEndPoint.fTime = latReference.nodes.at(endIndex).constraints.time;

			//GetStartDyn(m_sUSKEndPoint, m_sLongCur, m_sLatCur); // need a function for end point of the curve

			return tTrue;
		};

		/**
		 *  @brief generate Trajectory from FrenetSamples
		 *
		 *  @details the samples in \c i_psTrajSamplesS and \c i_psTrajSamplesD are concatenated by time and transformed back to USK
		 *  @details generated trajectory data gets appended to existing data specified by \c i_nStartSample.
		 *  @details because the generated Trajectory has a start time = 0 the time of the last valid trajectory-sample must be in \c i_fTimeAdd.
		 *  @details this sample defines the start dynamic for the Trajectory to append, so it will be overwritten.
		 *  @details if there is nothing to append to set \c i_fTimeAdd 0 or the current time.
		 *  @details reset the \c io_psCalculationData struct if the data in \c i_psTrajSamplesS changes
		 *
		 *  @param  i_asTrajSamplesS             samples of the longitudinal trajectory
		 *  @param  i_asTrajSamplesD             samples of the lateral trajectory
		 *  @param  i_nNoOfSamples               length of the sample arrays
		 *  @param  i_nNoOfOldPersistentSamples  number of samples the trajectory contains initially
		 *  @param  i_fTimeAdd                   time to add to the new generated points
		 *  @param  o_psTrajectory               Pointer to store the generated trajectory in
		 *  @param  io_rsCalculationData space to store intermediate results for next trajectory with the same \c i_psTrajSamplesS
		 *
		 */


		template <int noOfSamplesMax, typename TrajectoryType>
		tBool calculateVehDynamicsAlongTrajectory
		(IN const targetpoint::tSample             *i_asTrajSamplesS
			, IN const targetpoint::tSample             *i_asTrajSamplesD
			, IN const tUInt16                           i_nNoOfSamples
			, IN const tUInt16                           i_nNoOfOldPersistentSamples
			, IN const tFloat64                          i_fTimeAdd
			, OUT TrajectoryType						 &o_psTrajectory
			, INOUT tCalculationData<noOfSamplesMax>	 &io_sCalculationData
			, IN const tSampleFloat                      i_fDelta2Plan = 0.0f
			, IN const tBool i_bAbortOnFail = tFalse
		)
		{
			tBool nResult = tFalse;
			tUInt16 nSample;

			/// init the values of orientation and curvature to zero
			TL_tTrajPoint::value_type fLastPsi = 0;
			TL_tTrajPoint::value_type fLastC0 = 0;

			/// if there is a static part in the trajectory, we can use the information of the last point instead
			/// there should always be a static part, since even if the function is inactive, it is created artificially
			if (o_psTrajectory.nPoints > 0)
			{
				if (o_psTrajectory.nPoints <= TL_tTrajectoryRel::max_size())
				{
					fLastPsi = o_psTrajectory.aPoint[o_psTrajectory.nPoints - 1].fPsi;
					fLastC0 = o_psTrajectory.aPoint[o_psTrajectory.nPoints - 1].fC0;
				}
				else
				{
					return nResult;
				}
			}
			//STREAMLOG_INFO << "lastpsi = " << fLastPsi << std::endl;

			/// transformate point by point from frenet coordinate system to USK
			for (nSample = 0; ((nSample < i_nNoOfSamples) && (nSample + std::max(0, i_nNoOfOldPersistentSamples - 1) < o_psTrajectory.max_size())); ++nSample)
			{
				targetpoint::tSample sSampleS = i_asTrajSamplesS[nSample];
				sSampleS.afData[frenetspacelib::SAMPLE_POS] -= i_fDelta2Plan;
				/// transformate the current point
				// replace the error method with AMP method RETURN_IF_FAILED()
				nResult = frenetspacelib::FRT2USK(sSampleS, i_asTrajSamplesD[nSample], m_sRefPath, o_psTrajectory.aPoint[std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample], io_sCalculationData.asFRTvsUSK[nSample], io_sCalculationData.bCalculated, fLastPsi, fLastC0, io_sCalculationData.nLastIdx, i_bAbortOnFail);
				if (!nResult) {
					return nResult;
				}

				/// set time information
				o_psTrajectory.aPoint[std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample].fTime = (TL_tFloat)(i_asTrajSamplesS[nSample].fTime + i_fTimeAdd);
				/// save the current orientation and curvature
				fLastPsi = o_psTrajectory.aPoint[std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample].fPsi;
				fLastC0 = o_psTrajectory.aPoint[std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample].fC0;
				//             if (0 == nSample)
				//             {
				//                 logginglib::g_oInfoStream << ", psi = " << o_psTrajectory.aPoint[std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample].fPsi << ", vlong = " << o_psTrajectory.aPoint[__max(0, i_nNoOfOldPersistentSamples - 1) + nSample].fV << std::endl;
				//             }
			}

			/// save the total number of points
			o_psTrajectory.nPoints = std::max(0, i_nNoOfOldPersistentSamples - 1) + nSample;
			/// finished calculation without an error
			io_sCalculationData.bCalculated = tTrue;
			return nResult;
		}

		/**
		 *  @brief reset the calculation data struct
		 *
		 *  @details the calculation data needs to be reseted for each longitudinal trajectory
		 *
		 *  @param  io_rsCalculationData    reference of the calculation data struct
		 *
		 */
		template <int noOfSamplesMax>
		static tVoid resetCalculationData
		(tCalculationData<noOfSamplesMax> &io_sCalculationData)
		{
			io_sCalculationData.bCalculated = tFalse;
			io_sCalculationData.nLastIdx = 0;
		}

		tBool setReferencePath(IN const ltstReference_T &latReference)
		{
			tBool nResult = tFalse;
			m_sRefPath.nPoints = latReference.count;

			for (m_sRefPath.nPoints = 0; m_sRefPath.nPoints < latReference.count; ++m_sRefPath.nPoints)
			{
				m_sRefPath.aPoint[m_sRefPath.nPoints].fX = latReference.nodes.at(m_sRefPath.nPoints).constraints.posX;
				m_sRefPath.aPoint[m_sRefPath.nPoints].fY = latReference.nodes.at(m_sRefPath.nPoints).constraints.centerY;
				m_sRefPath.aPoint[m_sRefPath.nPoints].fPsi = latReference.nodes.at(m_sRefPath.nPoints).constraints.centerYaw;
				m_sRefPath.aPoint[m_sRefPath.nPoints].fC0 = latReference.nodes.at(m_sRefPath.nPoints).constraints.centerC0;
			}

			nResult = frenetspacelib::PrepareInternalData(m_sRefPath, tFalse, tTrue);

			return nResult;
		}

	};

}



