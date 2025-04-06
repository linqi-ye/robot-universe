import xml.etree.ElementTree as ET

def validate_xml(file_path):
    try:
        tree = ET.parse(file_path)
        print("XML is well-formed")
    except ET.ParseError as e:
        print(f"XML is not well-formed: {e}")

validate_xml('/home/ubuntu2004/kuavo/models/biped_gen5.0/urdf/biped_v5_all_joint.urdf')
