/******************************************************************************
  Copyright (c) 2008-2009 Ryan Juckett
  http://www.ryanjuckett.com/

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.

  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.

  3. This notice may not be removed or altered from any source
     distribution.
******************************************************************************/
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Diagnostics;
using UnityEngine;

namespace IK
{
	public class Solver
	{
		///***************************************************************************************
		/// SimplifyAngle
		/// This function will convert an angle to the equivalent rotation in the range [-pi,pi]
		///***************************************************************************************
		private static double SimplifyAngle (double angle)
		{
			angle = angle % (2.0 * Math.PI);
			if (angle < -Math.PI)
				angle += (2.0 * Math.PI);
			else if (angle > Math.PI)
				angle -= (2.0 * Math.PI);
			return angle;
		}

		///***************************************************************************************
		/// CalcIK_2D_TwoBoneAnalytic
		/// Given a two bone chain located at the origin (bone1 is the parent of bone2), this
		/// function will compute the bone angles needed for the end of the chain to line up
		/// with a target position. If there is no valid solution, the angles will be set to
		/// get as close to the target as possible.
		///  
		/// returns: True when a valid solution was found.
		///***************************************************************************************
		public static bool CalcIK_2D_TwoBoneAnalytic
		(
			out double angle1, // Angle of bone 1
			out double angle2, // Angle of bone 2
			bool solvePosAngle2, // Solve for positive angle 2 instead of negative angle 2
			double length1, // Length of bone 1. Assumed to be >= zero
			double length2, // Length of bone 2. Assumed to be >= zero
			double targetX, // Target x position for the bones to reach
			double targetY       // Target y position for the bones to reach
		)
		{
			//Debug.Assert (length1 >= 0);
			//Debug.Assert (length2 >= 0);

			const double epsilon = 0.0001; // used to prevent division by small numbers

			bool foundValidSolution = true;

			double targetDistSqr = (targetX * targetX + targetY * targetY);

			//===
			// Compute a new value for angle2 along with its cosine
			double sinAngle2;
			double cosAngle2;
	 
			double cosAngle2_denom = 2 * length1 * length2;
			if (cosAngle2_denom > epsilon) {
				cosAngle2 = (targetDistSqr - length1 * length1 - length2 * length2)
						/ (cosAngle2_denom);
	 
				// if our result is not in the legal cosine range, we can not find a
				// legal solution for the target
				if ((cosAngle2 < -1.0) || (cosAngle2 > 1.0))
					foundValidSolution = false;
	 
				// clamp our value into range so we can calculate the best
				// solution when there are no valid ones
				cosAngle2 = Math.Max (-1, Math.Min (1, cosAngle2));
	 
				// compute a new value for angle2
				angle2 = Math.Acos (cosAngle2);
	 
				// adjust for the desired bend direction
				if (!solvePosAngle2)
					angle2 = -angle2;

				// compute the sine of our angle
				sinAngle2 = Math.Sin (angle2);
			} else {
				// At leaset one of the bones had a zero length. This means our
				// solvable domain is a circle around the origin with a radius
				// equal to the sum of our bone lengths.
				double totalLenSqr = (length1 + length2) * (length1 + length2);
				if (targetDistSqr < (totalLenSqr - epsilon)
				|| targetDistSqr > (totalLenSqr + epsilon)) {
					foundValidSolution = false;
				}

				// Only the value of angle1 matters at this point. We can just
				// set angle2 to zero. 
				angle2 = 0.0;
				cosAngle2 = 1.0;
				sinAngle2 = 0.0;
			}
	 
			//===
			// Compute the value of angle1 based on the sine and cosine of angle2
			double triAdjacent = length1 + length2 * cosAngle2;
			double triOpposite = length2 * sinAngle2;
	 
			double tanY = targetY * triAdjacent - targetX * triOpposite;
			double tanX = targetX * triAdjacent + targetY * triOpposite;
	 
			// Note that it is safe to call Atan2(0,0) which will happen if targetX and
			// targetY are zero
			angle1 = Math.Atan2 (tanY, tanX);
	 
			return foundValidSolution;
		}

		///***************************************************************************************
		/// CalcIK_2D_AllTwoBoneAnalytic
		/// Given a two bone chain located at the origin (bone1 is the parent of bone2), this
		/// function will compute both possible solutions of bone angles needed for the end of
		/// the chain to line up with a target position. If there is no valid solution, the
		/// angles will be set to get as close to the target as possible.
		///  
		/// returns: True when a valid solution was found.
		///***************************************************************************************
		public static bool CalcIK_2D_AllTwoBoneAnalytic
		(
			out double solution1_angle1, // Angle of bone 1
			out double solution1_angle2, // Angle of bone 2
			out double solution2_angle1, // Angle of bone 1
			out double solution2_angle2, // Angle of bone 2
			double length1, // Length of bone 1. Assumed to be >= zero.
			double length2, // Length of bone 2. Assumed to be >= zero.
			double targetX, // Target x position for the bones to reach.
			double targetY               // Target y position for the bones to reach.
		)
		{
			//Debug.Assert (length1 >= 0);
			//Debug.Assert (length2 >= 0);

			const double epsilon = 0.0001; // used to prevent division by small numbers
	 
			bool foundValidSolution = true;

			double targetDistSqr = (targetX * targetX + targetY * targetY);

			//===
			// Compute a new value for angle2 along with its cosine
			double sinAngle2;
			double cosAngle2;
	 
			double cosAngle2_denom = 2 * length1 * length2;
			if (cosAngle2_denom > epsilon) {
				cosAngle2 = (targetDistSqr - length1 * length1 - length2 * length2)
						/ (cosAngle2_denom);
	 
				// if our result is not in the legal cosine range, we can not find a
				// legal solution for the target
				if ((cosAngle2 < -1.0) || (cosAngle2 > 1.0))
					foundValidSolution = false;
	 
				// clamp our value into range because of the epsilon range test
				cosAngle2 = Math.Max (-1, Math.Min (1, cosAngle2));
	 
				// compute a new value for angle2
				solution1_angle2 = Math.Acos (cosAngle2);

				// compute the sine of our angle
				sinAngle2 = Math.Sin (solution1_angle2);
			} else {
				// At leaset one of the bones had a zero length. This means our
				// solvable domain is a circle around the origin with a radius
				// equal to the sum of our bone lengths.
				double totalLenSqr = (length1 + length2) * (length1 + length2);
				if (targetDistSqr < (totalLenSqr - epsilon) || targetDistSqr > (totalLenSqr + epsilon))
					foundValidSolution = false;

				// Only the value of angle 1 matters at this point. We can just
				// set angle2 to zero. 
				solution1_angle2 = 0.0;
				cosAngle2 = 1.0;
				sinAngle2 = 0.0;
			}
	 
			//===
			// Compute the value of angle1 based on the sine and cosine of angle2
			double triAdjacent = length1 + length2 * cosAngle2;
			double triOpposite = length2 * sinAngle2;
	 
			double tanY = targetY * triAdjacent - targetX * triOpposite;
			double tanX = targetX * triAdjacent + targetY * triOpposite;
	 
			// Note that it is safe to call Atan2(0,0) which will happen if targetX and
			// targetY are zero
			solution1_angle1 = Math.Atan2 (targetY * triAdjacent - targetX * triOpposite, targetX * triAdjacent + targetY * triOpposite);

			solution2_angle2 = -solution1_angle2;
			solution2_angle1 = Math.Atan2 (targetY * triAdjacent + targetX * triOpposite, targetX * triAdjacent - targetY * triOpposite);
	 
			return foundValidSolution;
		}

		///***************************************************************************************
		/// Bone_2D_CCD_World
		/// This class is used internally by the CalcIK_2D_CCD function to represent a bone in
		/// world space.
		///***************************************************************************************
		private class Bone_2D_CCD_World
		{
			public double x;        // x position in world space
			public double y;        // y position in world space
			public double angle;    // angle in world space
			public double cosAngle; // sine of angle
			public double sinAngle; // cosine of angle
		};

		///***************************************************************************************
		/// CCDResult
		/// This enum represents the resulting state of a CCD iteration.
		///***************************************************************************************
		public enum CCDResult
		{
			Success,    // the target was reached
			Processing, // still trying to reach the target
			Failure,    // failed to reach the target
		}

		///***************************************************************************************
		/// Bone_2D_CCD
		/// This class is used to supply the CalcIK_2D_CCD function with a bone's representation
		/// relative to its parent in the kinematic chain.
		///***************************************************************************************
		public class Bone_2D_CCD
		{
			public double x;     // x position in parent space
			public double y;     // y position in parent space
			public double angle; // angle in parent space
		};

		///***************************************************************************************
		/// CalcIK_2D_CCD
		/// Given a bone chain located at the origin, this function will perform a single cyclic
		/// coordinate descent (CCD) iteration. This finds a solution of bone angles that places
		/// the final bone in the given chain at a target position. The supplied bone angles are
		/// used to prime the CCD iteration. If a valid solution does not exist, the angles will
		/// move as close to the target as possible. The user should resupply the updated angles 
		/// until a valid solution is found (or until an iteration limit is met).
		///  
		/// returns: CCDResult.Success when a valid solution was found.
		///          CCDResult.Processing when still searching for a valid solution.
		///          CCDResult.Failure when it can get no closer to the target.
		///***************************************************************************************
		public static CCDResult CalcIK_2D_CCD
		(
			ref List<Bone_2D_CCD> bones, // Bone values to update
			double targetX, // Target x position for the end effector
			double targetY, // Target y position for the end effector
			double arrivalDist           // Must get within this range of the target
		)
		{
			// Set an epsilon value to prevent division by small numbers.
			const double epsilon = 0.0001; 

			// Set max arc length a bone can move the end effector an be considered no motion
			// so that we can detect a failure state.
			const double trivialArcLength = 0.00001; 


			int numBones = bones.Count;
			//Debug.Assert (numBones > 0);

			double arrivalDistSqr = arrivalDist * arrivalDist;

			//===
			// Generate the world space bone data.
			List<Bone_2D_CCD_World> worldBones = new List<Bone_2D_CCD_World> ();

			// Start with the root bone.
			Bone_2D_CCD_World rootWorldBone = new Bone_2D_CCD_World ();
			rootWorldBone.x = bones [0].x;
			rootWorldBone.y = bones [0].y;
			rootWorldBone.angle = bones [0].angle;
			rootWorldBone.cosAngle = Math.Cos (rootWorldBone.angle);
			rootWorldBone.sinAngle = Math.Sin (rootWorldBone.angle);
			worldBones.Add (rootWorldBone);
		
			// Convert child bones to world space.
			for (int boneIdx = 1; boneIdx < numBones; ++boneIdx) {
				Bone_2D_CCD_World prevWorldBone = worldBones [boneIdx - 1];
				Bone_2D_CCD curLocalBone = bones [boneIdx];

				Bone_2D_CCD_World newWorldBone = new Bone_2D_CCD_World ();
				newWorldBone.x = prevWorldBone.x + prevWorldBone.cosAngle * curLocalBone.x
			                                 - prevWorldBone.sinAngle * curLocalBone.y;
				newWorldBone.y = prevWorldBone.y + prevWorldBone.sinAngle * curLocalBone.x
			                                 + prevWorldBone.cosAngle * curLocalBone.y;
				newWorldBone.angle = prevWorldBone.angle + curLocalBone.angle;
				newWorldBone.cosAngle = Math.Cos (newWorldBone.angle);
				newWorldBone.sinAngle = Math.Sin (newWorldBone.angle);
				worldBones.Add (newWorldBone);
			}
		
			//===
			// Track the end effector position (the final bone)
			double endX = worldBones [numBones - 1].x;
			double endY = worldBones [numBones - 1].y;

			//===
			// Perform CCD on the bones by optimizing each bone in a loop 
			// from the final bone to the root bone
			bool modifiedBones = false;
			for (int boneIdx = numBones-2; boneIdx >= 0; --boneIdx) {
				// Get the vector from the current bone to the end effector position.
				double curToEndX = endX - worldBones [boneIdx].x;
				double curToEndY = endY - worldBones [boneIdx].y;
				double curToEndMag = Math.Sqrt (curToEndX * curToEndX + curToEndY * curToEndY);

				// Get the vector from the current bone to the target position.
				double curToTargetX = targetX - worldBones [boneIdx].x;
				double curToTargetY = targetY - worldBones [boneIdx].y;
				double curToTargetMag = Math.Sqrt (curToTargetX * curToTargetX
			                                   + curToTargetY * curToTargetY);

				// Get rotation to place the end effector on the line from the current
				// joint position to the target postion.
				double cosRotAng;
				double sinRotAng;
				double endTargetMag = (curToEndMag * curToTargetMag);
				if (endTargetMag <= epsilon) {
					cosRotAng = 1;
					sinRotAng = 0;
				} else {
					cosRotAng = (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag;
					sinRotAng = (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag;
				}

				// Clamp the cosine into range when computing the angle (might be out of range
				// due to floating point error).
				double rotAng = Math.Acos (Math.Max (-1, Math.Min (1, cosRotAng)));
				if (sinRotAng < 0.0)
					rotAng = -rotAng;
			
				// Rotate the end effector position.
				endX = worldBones [boneIdx].x + cosRotAng * curToEndX - sinRotAng * curToEndY;
				endY = worldBones [boneIdx].y + sinRotAng * curToEndX + cosRotAng * curToEndY;

				// Rotate the current bone in local space (this value is output to the user)
				bones [boneIdx].angle = SimplifyAngle (bones [boneIdx].angle + rotAng);

				// Check for termination
				double endToTargetX = (targetX - endX);
				double endToTargetY = (targetY - endY);
				if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= arrivalDistSqr) {
					// We found a valid solution.
					return CCDResult.Success;
				}

				// Track if the arc length that we moved the end effector was
				// a nontrivial distance.
				if (!modifiedBones && Math.Abs (rotAng) * curToEndMag > trivialArcLength) {
					modifiedBones = true;
				}
			}

			// We failed to find a valid solution during this iteration.
			if (modifiedBones)
				return CCDResult.Processing;
			else
				return CCDResult.Failure;
		}

		///***************************************************************************************
		/// Bone_2D_ConstraintRelaxation
		/// This class is used to supply the constraint relaxation functions with a bone's
		/// representation relative to its parent in the kinematic chain.
		///***************************************************************************************
		public class Bone_2D_ConstraintRelaxation
		{
			public double angle;  // angle in parent space
			public double length; // length of the bone (relaxation constraint)
			public double weight; // weight of the bone during relaxation
		};

		///***************************************************************************************
		/// Bone_2D_ConstraintRelaxation_World
		/// This class is used to supply the constraint relaxation functions with a bone's
		/// representation in world space.
		///***************************************************************************************
		public class Bone_2D_ConstraintRelaxation_World
		{
			public double x;      // x position in world space
			public double y;      // y position in world space
			public double length; // length of the bone (relaxation constraint)
			public double weight; // weight of the bone during relaxation
		};

		///***************************************************************************************
		/// CalcIK_2D_ConstraintRelaxation_ConvertToWorld
		/// This is a helper function to generate a world space bone chain for relaxation given
		/// a local space bone chain.
		///***************************************************************************************
		public static void CalcIK_2D_ConstraintRelaxation_ConvertToWorld
		(
			out List<Bone_2D_ConstraintRelaxation_World> worldBones, // Output world space bones
			List<Bone_2D_ConstraintRelaxation>           localBones  // Input local space bones
		)
		{
			int numBones = localBones.Count;
			//Debug.Assert (numBones > 0);

			// create the list to output
			worldBones = new List<Bone_2D_ConstraintRelaxation_World> ();

			// Start with the root bone.
			Bone_2D_ConstraintRelaxation_World rootWorldBone
			= new Bone_2D_ConstraintRelaxation_World ();
			rootWorldBone.x = 0;
			rootWorldBone.y = 0;
			rootWorldBone.length = localBones [0].length;
			rootWorldBone.weight = localBones [0].weight;
			worldBones.Add (rootWorldBone);
	    
			double prevAngle = localBones [0].angle;
			double prevAngleCos = Math.Cos (prevAngle);
			double prevAngleSin = Math.Sin (prevAngle);
	    
			// Convert child bones to world space.
			for (int boneIdx = 1; boneIdx < numBones; ++boneIdx) {
				Bone_2D_ConstraintRelaxation_World prevWorldBone = worldBones [boneIdx - 1];
				Bone_2D_ConstraintRelaxation prevLocalBone = localBones [boneIdx - 1];

				Bone_2D_ConstraintRelaxation_World newWorldBone
				= new Bone_2D_ConstraintRelaxation_World ();
				newWorldBone.x = prevWorldBone.x + prevAngleCos * prevLocalBone.length;
				newWorldBone.y = prevWorldBone.y + prevAngleSin * prevLocalBone.length;
				newWorldBone.length = localBones [boneIdx].length;
				newWorldBone.weight = localBones [boneIdx].weight;
				worldBones.Add (newWorldBone);

				prevAngle = prevAngle + localBones [boneIdx].angle;
				prevAngleCos = Math.Cos (prevAngle);
				prevAngleSin = Math.Sin (prevAngle);
			}
		}

		///***************************************************************************************
		/// CalcIK_2D_ConstraintRelaxation
		/// Given a bone chain located at the origin, this function will perform a single
		/// relaxation iteration. This finds a solution of bone angles that places the final bone
		/// in the given chain at a target position. The supplied bone angles are used to prime
		/// the iteration. If a valid solution does not exist, the angles will move as close to
		/// the target as possible. The user should resupply the updated angles until a valid
		/// solution is found (or until an iteration limit is met).
		///***************************************************************************************
		public static void CalcIK_2D_ConstraintRelaxation
		(
			ref List<Bone_2D_ConstraintRelaxation_World> bones, // Bone values to update
			double targetX, // Target x position for the end effector
			double targetY                            // Target y position for the end effector
		)
		{
			// Set an epsilon value to prevent division by small numbers.
			const double epsilon = 0.0001;

			int numBones = bones.Count;
			//Debug.Assert (numBones > 0);

			//===
			// Constrain the end bone to the target.
			{
				int boneIdx = numBones - 1;
				double toTargetX = targetX - bones [boneIdx].x;
				double toTargetY = targetY - bones [boneIdx].y;
				double toTargetLenSqr = toTargetX * toTargetX + toTargetY * toTargetY;
				if (toTargetLenSqr > epsilon) {
					double toTargetLen = Math.Sqrt (toTargetLenSqr);
					double toTargetScale = (bones [boneIdx].length / toTargetLen) - 1.0;
					bones [boneIdx].x -= toTargetScale * toTargetX;
					bones [boneIdx].y -= toTargetScale * toTargetY;
				}
			}

			//===
			// Perform relaxation on the bones in a loop from the final bone to the first child
			// bone.
			for (int boneIdx = numBones-2; boneIdx >= 1; --boneIdx) {
				Bone_2D_ConstraintRelaxation_World curBone = bones [boneIdx];
				Bone_2D_ConstraintRelaxation_World childBone = bones [boneIdx + 1];

				// Get the vector from the current bone to the child bone.
				double toChildX = childBone.x - curBone.x;
				double toChildY = childBone.y - curBone.y;

				double toChildLenSqr = toChildX * toChildX + toChildY * toChildY;
				double totalWeight = curBone.weight + childBone.weight;
				if (toChildLenSqr > epsilon && totalWeight > epsilon) {
					double toChildLen = Math.Sqrt (toChildLenSqr);
					double toChildScale =
					  ((bones [boneIdx].length / toChildLen) - 1.0) / totalWeight;
					double curBoneScale = toChildScale * curBone.weight;
					double childBoneScale = toChildScale * childBone.weight;

					curBone.x -= curBoneScale * toChildX;
					curBone.y -= curBoneScale * toChildY;

					childBone.x += childBoneScale * toChildX;
					childBone.y += childBoneScale * toChildY;
				}
			}

			//===
			// Constrain the first child joint to the root joint
			if (numBones > 1) {
				int boneIdx = 0;

				// Get the vector from the current bone to the child bone.
				double toChildX = bones [boneIdx + 1].x - bones [boneIdx].x;
				double toChildY = bones [boneIdx + 1].y - bones [boneIdx].y;

				double toChildLenSqr = toChildX * toChildX + toChildY * toChildY;
				if (toChildLenSqr > epsilon) {
					double toChildLen = Math.Sqrt (toChildLenSqr);
					double toChildScale = (bones [boneIdx].length / toChildLen) - 1.0;

					bones [boneIdx + 1].x += toChildScale * toChildX;
					bones [boneIdx + 1].y += toChildScale * toChildY;
				}
			}
		}


		///***************************************************************************************
		/// CalcIK_2D_ConstraintRelaxation_ConvertToLocal
		/// This is a helper function to update a local space bone chain after relaxation has been
		/// performed on the corresponding world space bone chain.
		///***************************************************************************************
		public static void CalcIK_2D_ConstraintRelaxation_ConvertToLocal
		(
			ref List<Bone_2D_ConstraintRelaxation> localBones, // Update local space bones
			List<Bone_2D_ConstraintRelaxation_World> worldBones, // Input world space bones
			double targetX, // Target x position for the end effector
			double targetY                             // Target y position for the end effector
		)
		{
			//Debug.Assert (localBones.Count == worldBones.Count);

			int numBones = localBones.Count;
			//Debug.Assert (numBones > 0);

			// Extract bone angles for all bones but the end bone
			double prevWorldAngle = 0.0;
			for (int boneIdx = 0; boneIdx < numBones-1; ++boneIdx) {
				double toNextBoneX = worldBones [boneIdx + 1].x - worldBones [boneIdx].x;
				double toNextBoneY = worldBones [boneIdx + 1].y - worldBones [boneIdx].y;
				double worldAngle = Math.Atan2 (toNextBoneY, toNextBoneX);

				double newAngle = SimplifyAngle (worldAngle - prevWorldAngle);

				localBones [boneIdx].angle = newAngle;

				prevWorldAngle = worldAngle;
			}

			// Point the end bone towards the target
			{
				int boneIdx = numBones - 1;
				double toTargetX = targetX - worldBones [boneIdx].x;
				double toTargetY = targetY - worldBones [boneIdx].y;
				double worldAngle = Math.Atan2 (toTargetY, toTargetX);

				double newAngle = SimplifyAngle (worldAngle - prevWorldAngle);

				localBones [boneIdx].angle = newAngle;
			}
		}

		#region Internal types
		
		// this class represents a bone in it's parent space
		public class BoneData
		{
			// actual bone data
			private double m_length = 0;
			private double m_angle = 0;
			private double m_weight = 0;

			// positions used to display the relaxation process
			private double m_relaxedX = 0;
			private double m_relaxedY = 0;
	
			public double Length {
				get { return m_length; }
				set { m_length = value; }
			}

			public double Radians { 
				get { return m_angle; }
				set { m_angle = value; }
			}

			public double Degrees { 
				get { return m_angle * 180.0 / Math.PI; }
				set { m_angle = value * Math.PI / 180.0; }
			}

			public double Weight {
				get { return m_weight; }
				set { m_weight = value; }
			}

			public double RelaxedX {
				get { return m_relaxedX; }
				set { m_relaxedX = value; }
			}

			public double RelaxedY {
				get { return m_relaxedY; }
				set { m_relaxedY = value; }
			}
		}
		
		#endregion

		#region Private data

		// number of iterations to perform on each update
		private int m_iterationsPerUpdate = 0;

		// bone lengths
		private Collection<BoneData> m_bones = new Collection<BoneData> ();
		
		// lines used to draw the bones
		class BoneLine
		{
//			public Line m_actualBone = null;
//			public Line m_relaxedBone = null;
		}
		private List<BoneLine> m_boneLines = new List<BoneLine> ();
		
		// target position to reach for
		private Vector2 m_targetPos = new Vector2 (0, 0);

		// positions used to display the relaxation process
		private double m_relaxedTargetX = 0;
		private double m_relaxedTargetY = 0;

		//private DispatcherTimer m_updateTimer;

		#endregion

		#region Public properties

		public Collection<BoneData> Bones {
			get { return m_bones; }
		}

		public int IterationsPerUpdate {
			get { return m_iterationsPerUpdate; }
			set {
				m_iterationsPerUpdate = Math.Max (1, value);
			}
		}

		public float TargetPosX {
			get { return m_targetPos.x; }
			set {
				m_targetPos.x = value;
				UpdateDisplay (); // redraw
			}
		}

		public float TargetPosY {
			get { return m_targetPos.y; }
			set {
				m_targetPos.y = value;
				UpdateDisplay (); // redraw
			}
		}

		#endregion

		#region Lifespan functions
		public Solver ()
		{		
			// set the iteration number
			IterationsPerUpdate = 1;

			// create the timer
			//m_updateTimer = new DispatcherTimer(DispatcherPriority.Normal);
			//m_updateTimer.Tick += new EventHandler(UpdateTimer_Tick);
			//m_updateTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);

			// add the initial bones
			AddBone ();
			AddBone ();

			TargetPosX = 100;
			TargetPosY = 0;

			// update the display
			UpdateDisplay ();
		}
		#endregion
		
		#region Coordinate Conversion

		// compute the logical origin in _viewport coordinated
		private double ViewportWidth { get { return Screen.width; } }

		private double ViewportHieght { get { return Screen.height; } }

		private double ViewportCenterX { get { return ViewportWidth / 2; } }

		private double ViewportCenterY { get { return ViewportHieght / 2; } }

		// convert logical coordinates to _viewport coordinates
		private double LogicalToViewportX (double logicalX)
		{
			return logicalX + ViewportCenterX;
		}

		private double LogicalToViewportY (double logicalY)
		{
			return -logicalY + ViewportCenterY;
		}
	
		// convert _viewport coordinates to logical coordinates
		private double ViewportToLogicalX (double viewportX)
		{
			return viewportX - ViewportCenterX;
		}

		private double ViewportToLogicalY (double viewportY)
		{
			return -viewportY + ViewportCenterY;
		}
		
		#endregion

		#region Logic Functions

		/// <summary>
		/// Add a new bone to the chain at the selected location or at the end if no location is selected.
		/// </summary>
		void AddBone ()
		{
			BoneData newBone = new BoneData ();
			newBone.Length = 50;
			newBone.Weight = 1;

			// insert at the end if no bone is selected
			//if (_boneList.SelectedIndex == -1)
				Bones.Add (newBone);
			//else
			//	Bones.Insert (_boneList.SelectedIndex, newBone);
		}

		/// <summary>
		/// Remove a new bone from the chain at the selected location or from the end if no location is selected.
		/// </summary>
		private void RemoveBone ()
		{
			if (Bones.Count == 0)
				return;

			// remove the end bone if no bone is selected
			int removeIdx;// = _boneList.SelectedIndex;
//			if (removeIdx == -1)
				removeIdx = (Bones.Count - 1);

			Bones.RemoveAt (removeIdx);
		}

		/// <summary>
		/// Perform an iteration of IK
		/// </summary>
		private void UpdateIK ()
		{
			int numBones = Bones.Count;
			
			if (numBones == 0)
				return;

			// calculate the bone angles
			List< Bone_2D_ConstraintRelaxation > relaxBones = new List< Bone_2D_ConstraintRelaxation > ();
			for (int boneIdx = 0; boneIdx < numBones; ++boneIdx) {
				Bone_2D_ConstraintRelaxation newRelaxBone = new Bone_2D_ConstraintRelaxation ();
				newRelaxBone.angle = Bones [boneIdx].Radians;
				newRelaxBone.length = Bones [boneIdx].Length;
				newRelaxBone.weight = Bones [boneIdx].Weight;
				relaxBones.Add (newRelaxBone);
			}

			// convert to worldspace bones
			List< Bone_2D_ConstraintRelaxation_World > worldBones;
			CalcIK_2D_ConstraintRelaxation_ConvertToWorld (out worldBones, relaxBones);

			// iterate IK
			for (int itrCount = 0; itrCount < IterationsPerUpdate; ++itrCount) {
				CalcIK_2D_ConstraintRelaxation (ref worldBones, TargetPosX, TargetPosY);
			}

			// convert bones back to local space
			CalcIK_2D_ConstraintRelaxation_ConvertToLocal (ref relaxBones, worldBones, TargetPosX, TargetPosY);
			
			// extract the new bone data from the results
			for (int boneIdx = 0; boneIdx < numBones; ++boneIdx) {
				Bones [boneIdx].Radians = relaxBones [boneIdx].angle;
				Bones [boneIdx].RelaxedX = worldBones [boneIdx].x;
				Bones [boneIdx].RelaxedY = worldBones [boneIdx].y;
			}

			// track the target position we used for relaxation (this prevents the relaxed bone display from changing while paused)
			m_relaxedTargetX = TargetPosX;
			m_relaxedTargetY = TargetPosY;
		}

		/// <summary>
		/// Update the scene displayed in the viewport
		/// </summary>
		private void UpdateDisplay ()
		{
			/*
			int numBones = Bones.Count;
			
			// resize the number of bone lines
			while (m_boneLines.Count > numBones) {
				_viewport.Children.Remove (m_boneLines [m_boneLines.Count - 1].m_actualBone);
				_viewport.Children.Remove (m_boneLines [m_boneLines.Count - 1].m_relaxedBone);
				m_boneLines.RemoveAt (m_boneLines.Count - 1);
			}

			while (m_boneLines.Count < numBones) {
				BoneLine newBoneLine = new BoneLine ();
				newBoneLine.m_actualBone = new Line ();
				newBoneLine.m_actualBone.Stroke = s_boneLineColors [m_boneLines.Count % s_boneLineColors.Length];
				newBoneLine.m_actualBone.StrokeThickness = 3;
				newBoneLine.m_actualBone.Opacity = 0.85;
				newBoneLine.m_actualBone.SetValue (Panel.ZIndexProperty, 100);

				newBoneLine.m_relaxedBone = new Line ();
				newBoneLine.m_relaxedBone.Stroke = s_boneLineColors [m_boneLines.Count % s_boneLineColors.Length];
				newBoneLine.m_relaxedBone.StrokeThickness = 7;
				newBoneLine.m_relaxedBone.Opacity = 0.15;
				newBoneLine.m_relaxedBone.SetValue (Panel.ZIndexProperty, 99);

				m_boneLines.Add (newBoneLine);
				_viewport.Children.Add (newBoneLine.m_actualBone);
				_viewport.Children.Add (newBoneLine.m_relaxedBone);
			}

			// compute the orientations of the bone lines in logical space
			double curAngle = 0;
			for (int boneIdx = 0; boneIdx < numBones; ++boneIdx) {
				BoneData curBone = Bones [boneIdx];

				curAngle += curBone.Radians;
				double cosAngle = Math.Cos (curAngle);
				double sinAngle = Math.Sin (curAngle);

				if (boneIdx > 0) {
					m_boneLines [boneIdx].m_actualBone.X1 = m_boneLines [boneIdx - 1].m_actualBone.X2;
					m_boneLines [boneIdx].m_actualBone.Y1 = m_boneLines [boneIdx - 1].m_actualBone.Y2;
				} else {
					m_boneLines [boneIdx].m_actualBone.X1 = 0;
					m_boneLines [boneIdx].m_actualBone.Y1 = 0;
				}

				m_boneLines [boneIdx].m_actualBone.X2 = m_boneLines [boneIdx].m_actualBone.X1 + cosAngle * curBone.Length;
				m_boneLines [boneIdx].m_actualBone.Y2 = m_boneLines [boneIdx].m_actualBone.Y1 + sinAngle * curBone.Length;

				m_boneLines [boneIdx].m_relaxedBone.X1 = curBone.RelaxedX;
				m_boneLines [boneIdx].m_relaxedBone.Y1 = curBone.RelaxedY;

				if (boneIdx < numBones - 1) {
					m_boneLines [boneIdx].m_relaxedBone.X2 = Bones [boneIdx + 1].RelaxedX;
					m_boneLines [boneIdx].m_relaxedBone.Y2 = Bones [boneIdx + 1].RelaxedY;
				} else {
					m_boneLines [boneIdx].m_relaxedBone.X2 = m_relaxedTargetX;
					m_boneLines [boneIdx].m_relaxedBone.Y2 = m_relaxedTargetY;
				}
			}

			// convert the bone positions to viewport space
			foreach (BoneLine curLine in m_boneLines) {
				curLine.m_actualBone.X1 = LogicalToViewportX (curLine.m_actualBone.X1);
				curLine.m_actualBone.Y1 = LogicalToViewportY (curLine.m_actualBone.Y1);

				curLine.m_actualBone.X2 = LogicalToViewportX (curLine.m_actualBone.X2);
				curLine.m_actualBone.Y2 = LogicalToViewportY (curLine.m_actualBone.Y2);

				curLine.m_relaxedBone.X1 = LogicalToViewportX (curLine.m_relaxedBone.X1);
				curLine.m_relaxedBone.Y1 = LogicalToViewportY (curLine.m_relaxedBone.Y1);

				curLine.m_relaxedBone.X2 = LogicalToViewportX (curLine.m_relaxedBone.X2);
				curLine.m_relaxedBone.Y2 = LogicalToViewportY (curLine.m_relaxedBone.Y2);
			}

			// draw the target
			Canvas.SetLeft (_targetEllipse, LogicalToViewportX (TargetPosX - _targetEllipse.Width / 2));
			Canvas.SetTop (_targetEllipse, LogicalToViewportY (TargetPosY + _targetEllipse.Height / 2));
			
			// draw the axes
			_xAxisLine.X1 = 0;
			_xAxisLine.Y1 = ViewportCenterY;
			_xAxisLine.X2 = ViewportWidth;
			_xAxisLine.Y2 = ViewportCenterY;

			_yAxisLine.X1 = ViewportCenterX;
			_yAxisLine.Y1 = 0;
			_yAxisLine.X2 = ViewportCenterX;
			_yAxisLine.Y2 = ViewportHieght;
			*/
		}

		/// <summary>
		/// Update logic at a set interval
		/// </summary>
		private void UpdateTimer_Tick (object sender, EventArgs e)
		{
			UpdateIK ();
			UpdateDisplay ();
		}

		#endregion
		
		/*
		#region Event Handlers

		private void BonePropertyChanged(object sender, PropertyChangedEventArgs e)
		{
			switch (e.PropertyName)
			{
				case "Radians":
					UpdateDisplay();
					break;
				case "Length":
					UpdateDisplay();
					break;
			}
		}

		private void viewport_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			// capture the mouse to keep grabing MouseMove events if the user drags the
			// mouse outside of the _viewport bounds
			if (!_viewport.IsMouseCaptured)
            {
                _viewport.CaptureMouse();
			}

			// update the target position
			Point viewportPos = e.GetPosition(_viewport);
			TargetPosX = ViewportToLogicalX( viewportPos.X );
			TargetPosY = ViewportToLogicalY( viewportPos.Y );
		}

		private void viewport_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			// release the captured mouse
			if (_viewport.IsMouseCaptured)
            {
                _viewport.ReleaseMouseCapture();
            }
		}

		private void viewport_MouseMove(object sender, MouseEventArgs e)
		{
			// update the target position if we are still in a captured state
			// (i.e. the user has not released the mouse button)
			if (_viewport.IsMouseCaptured)
            {
				Point viewportPos = e.GetPosition(_viewport);
				TargetPosX = ViewportToLogicalX( viewportPos.X );
				TargetPosY = ViewportToLogicalY( viewportPos.Y );
            }
		}

		private void _thisWindow_SizeChanged(object sender, SizeChangedEventArgs e)
		{
			// update the display shapes based on the new window size
			UpdateDisplay();
		}
		
		private void _thisWindow_IsVisibleChanged(object sender, DependencyPropertyChangedEventArgs e)
		{			
			if( m_updateTimer != null )
			{
				if( this.IsVisible )
				{
					if( _playRadioButton.IsChecked == true )
						m_updateTimer.Start();
				}
				else
				{
					m_updateTimer.Stop();
				}
			}
		}

		private void _websiteLink_Click(object sender, RoutedEventArgs e)
		{
			System.Diagnostics.Process.Start( "http://www.ryanjuckett.com" );
		}

		private void _addBoneButton_Click(object sender, RoutedEventArgs e)
		{
			AddBone();
			UpdateDisplay();
		}

		private void _removeBoneButton_Click(object sender, RoutedEventArgs e)
		{
			RemoveBone();
			UpdateDisplay();
		}	

		private void _playRadioButton_Checked(object sender, RoutedEventArgs e)
		{
			//if( m_updateTimer != null )
			//	m_updateTimer.Start();
		}

		private void _pauseRadioButton_Checked(object sender, RoutedEventArgs e)
		{
			//if( m_updateTimer != null )
			//	m_updateTimer.Stop();
		}

		private void _singleUpdateButton_Click(object sender, RoutedEventArgs e)
		{
			_pauseRadioButton.IsChecked = true;
			UpdateIK();
			UpdateDisplay();
		}

		#endregion
		*/
	}
	
}