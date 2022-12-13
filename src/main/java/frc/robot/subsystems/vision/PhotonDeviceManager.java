package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonDeviceManager {

    public LinkedList<PhotonCamera> photonDevices;

    //Singleton functionality
    private static PhotonDeviceManager instance;
    public static PhotonDeviceManager getInstance(){
        return (instance == null) ? new PhotonDeviceManager() : instance;
    }

    private PhotonDeviceManager(){
        photonDevices = new LinkedList<PhotonCamera>();
    }
    
    public void addDevices(PhotonCamera ... c){
        Arrays.asList(c).stream().filter(cam -> photonDevices.contains(cam)).forEach(cam -> photonDevices.add(cam));
    }

    public PhotonCamera getSpecificCamera(int index){
        return photonDevices.stream().filter(t -> index == (t.getPipelineIndex())).findAny().orElse(null);
    }

    /**
     * Takes an image from the input stream and saves it to /opt/photonvision/photonvision_config/imgSaves
     * @param cameraIndex Index of the camera to take the snapshot of
     */
    public void takeInputSnapshot(int cameraIndex){
        getSpecificCamera(cameraIndex).takeInputSnapshot();
    }

    /**
     * Takes an image from the output stream and saves it to /opt/photonvision/photonvision_config/imgSaves
     * @param cameraIndex Index of the camera to take the snapshot of
     */
    public void takeOutputSnapshot(int cameraIndex){
        getSpecificCamera(cameraIndex).takeOutputSnapshot();
    }

    /**
     * Method to get the latest result of a photon camera
     * @param cameraIndex The camera from which the result is returned
     * @return The result provided by the camera
     */
    public PhotonPipelineResult getResult(int cameraIndex){
        return getSpecificCamera(cameraIndex).getLatestResult();
    }

    /**
     * Returns the best result of the camera, assuming this result is less than a max distance away.
     * @param cameraIndex Index of the camera to get the result from
     * @param maxAllowed The maximum allowed distance from the camera that still qualifies something as a "best target"
     * @return The best target less than a distance away
     */
    public PhotonTrackedTarget getBestResult(int cameraIndex, double maxAllowed){
        PhotonTrackedTarget potential = getResult(cameraIndex).getBestTarget();
        return (Math.sqrt(Math.pow(potential.getBestCameraToTarget().getX(), 2) +
                          Math.pow(potential.getBestCameraToTarget().getY(), 2) +
                          Math.pow(potential.getBestCameraToTarget().getZ(), 2)
                ) <= maxAllowed) ? potential : null;
    }

    /**
     * Returns the best result of the camera.
     * @param cameraIndex Index of the camera to get the result from
     * @return The best target returned by the camera
     */
    public PhotonTrackedTarget getBestResult(int cameraIndex){
        return getResult(cameraIndex).getBestTarget();
    }

    /**
     * Returns if the camera has a target or not.
     * @param cameraIndex Index of the camera to get the result from
     * @return If the camera detects a target or not
     */
    public boolean hasTarget(int cameraIndex){
        return getResult(cameraIndex).hasTargets();
    }

    /**
     * Returns if the camera has a target or not.
     * @param cameraIndex Index of the camera to get the result from
     * @return If the camera detects a target or not
     */
    public List<PhotonTrackedTarget> getAllTargets(int cameraIndex){
        return getResult(cameraIndex).getTargets();
    }
    

}
