// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera cam;
  AprilTagFieldLayout aprilTagFieldLayout;

  public PhotonVision() {
    cam = new PhotonCamera(VisionConstants.camName);
    try{
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); }
    catch(IOException IOE){
      IOE.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public PhotonPipelineResult getLatestResult(){
    return cam.getLatestResult();
  }
}
