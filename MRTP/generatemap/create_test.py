import struct
import os

# 1. Define the XML (Short enough to keep here)
xml_data = """<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>i need to sample trees 10 and 60</Mission>
  <BehaviorTree ID="Sample_Trees_10_60">
    <Sequence>
      <MoveToTreeID name="GoToTree10" action_name="follow_tree_id_waypoint" id="10" approach_tree="true"/>
      <SampleLeaf name="SampleTree10" action_name="segment_leaves"/>
      <MoveToTreeID name="ExitToTopRowCol2" action_name="follow_tree_id_waypoint" id="2" approach_tree="false"/>
      <MoveToTreeID name="TraverseTopRowToCol4" action_name="follow_tree_id_waypoint" id="4" approach_tree="false"/>
      <MoveToTreeID name="GoToTree60" action_name="follow_tree_id_waypoint" id="60" approach_tree="true"/>
      <SampleLeaf name="SampleTree60" action_name="segment_leaves"/>
    </Sequence>
  </BehaviorTree>
</root>"""

# 2. Read the JSON from the file you just created
try:
    with open("trees.json", "r") as f:
        json_data = f.read().strip()
    print(f"Loaded JSON data: {len(json_data)} bytes")
except FileNotFoundError:
    print("Error: 'trees.json' not found. Did you create it?")
    exit(1)

# 3. Pack the data [Size][XML][Size][JSON]
# !I = Network Byte Order (Big Endian) 4-byte Integer
xml_bytes = xml_data.encode('utf-8')
json_bytes = json_data.encode('utf-8')

packed_data = struct.pack('!I', len(xml_bytes)) + xml_bytes + \
              struct.pack('!I', len(json_bytes)) + json_bytes

# 4. Save to binary
filename = "create_test.bin"
with open(filename, 'wb') as f:
    f.write(packed_data)

print(f"Success! Created {filename}. You can now run:")
print(f"nc localhost 12346 < {filename}")