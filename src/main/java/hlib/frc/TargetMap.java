package hlib.frc;

import java.io.File;
import java.util.Iterator;
import java.util.LinkedHashMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import hlib.drive.Target;

/**
 * A {@code TargetMap} associates {@code Target}s with identifiers.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class TargetMap extends LinkedHashMap<String, Target> {

	/**
	 * The automatically generated serial version UID.
	 */
	private static final long serialVersionUID = -7059826741799599341L;

	/**
	 * Constructs a {@code TargetMap} by parsing the specified file.
	 * 
	 * @param fileName
	 *            the name of the file
	 */
	public TargetMap(String fileName) {
		try {
			JsonNode root = new ObjectMapper().readTree(new File(fileName));
			JsonNode targets = root.path("targets");
			Iterator<JsonNode> i = targets.elements();
			while (i.hasNext())
				update(i.next());
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println(size() + " targets read from \"" + fileName + "\".");
	}

	/**
	 * Updates this {@code TargetMap} based on the specified {@code JsonNode}.
	 * @param n a {@code JsonNode}
	 * @return the ID of the {@code Target} obtained from the specified {@code JsonNode}
	 */
	protected String update(JsonNode n) {
		String targetID = n.path("id").asText();
		Iterator<JsonNode> j = n.path("pose").elements();
		Target target = new Target(j.next().asDouble(), j.next().asDouble(), j.next().asDouble() * Math.PI / 180,
				n.path("distance threshold").asDouble(), n.path("angle threshold").asDouble() * Math.PI / 180,
				n.path("time threshold").asDouble());
		put(targetID, target);
		return targetID;
	}

}
