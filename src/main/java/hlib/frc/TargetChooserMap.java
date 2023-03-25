package hlib.frc;

import java.io.File;
import java.util.Iterator;
import java.util.LinkedHashMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * A {@code TargetChooserMap} associates each {@code Target} ID with a label.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class TargetChooserMap extends LinkedHashMap<String, String> {

	/**
	 * The automatically generated serial version UID.
	 */

	private static final long serialVersionUID = -5083647150936718395L;

	/**
	 * Constructs a {@code TargetChooserMap} by parsing the specified file.
	 * 
	 * @param fileName
	 *            the name of the file
	 */
	public TargetChooserMap(String fileName) {
		try {
			JsonNode root = new ObjectMapper().readTree(new File(fileName));
			JsonNode poses = root.path("target choosers");
			Iterator<JsonNode> i = poses.elements();
			while (i.hasNext()) {
				JsonNode node = i.next();
				String label = node.path("label").asText();
				String value = node.path("default value").asText();
				if (label != null && value != null)
					put(label, value);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println(size() + " target chooser labels read from \"" + fileName + "\".");
	}

}
