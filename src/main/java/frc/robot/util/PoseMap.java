package frc.robot.util;

import java.io.File;
import java.util.HashMap;
import java.util.Iterator;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import hlib.drive.Pose;

/**
 * A {@code PoseMap} associates {@code Pose}s with identifiers.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PoseMap extends HashMap<String, Pose> {

	/**
	 * The automatically generated serial version UID.
	 */
	private static final long serialVersionUID = -7059826741799599341L;

	/**
	 * Constructs a {@code PoseMap} by parsing the specified file.
	 * 
	 * @param fileName
	 *            the name of the file
	 */
	public PoseMap(String fileName) {
		try {
			JsonNode root = new ObjectMapper().readTree(new File(fileName));
			JsonNode poses = root.path("poses");
			Iterator<JsonNode> i = poses.elements();
			while (i.hasNext()) {
				JsonNode n = i.next();
				String poseID = n.path("id").asText();
				Iterator<JsonNode> j = n.path("pose").elements();
				Pose pose = new Pose(j.next().asDouble(), j.next().asDouble(), j.next().asDouble() * Math.PI / 180);
				put(poseID, pose);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println(size() + " poses read from \"" + fileName + "\".");
	}

}
