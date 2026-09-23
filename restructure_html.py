import re

with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/index.html', 'r') as f:
    html = f.read()

# Locate map section
map_start = html.find('<!-- Map Section -->')
map_end = html.find('</section>', map_start) + len('</section>')
map_html = html[map_start:map_end]

# Locate Dispatch Panel
dispatch_start = html.find('<!-- Dispatch Panel -->')
dispatch_end = html.find('<!-- System Controls -->')
dispatch_html = html[dispatch_start:dispatch_end]

# Locate System Controls
controls_start = html.find('<!-- System Controls -->')
controls_end = html.find('</div>', html.find('</button>', html.find('</button>', controls_start) + 1) + 1) + 6
controls_html = html[controls_start:controls_end]

# We need to extract dispatch and controls from where they are currently
html = html.replace(dispatch_html, '')
html = html.replace(controls_html, '')

# We will wrap the map and the controls in a left column
left_col_start = """
            <div class="left-column" style="display: flex; flex-direction: column; gap: 24px;">
"""
controls_wrapper_start = """
                <div class="bottom-controls" style="display: grid; grid-template-columns: 1fr 1fr; gap: 24px;">
"""
controls_wrapper_end = """
                </div>
"""
left_col_end = """
            </div>
"""

new_left_col = left_col_start + map_html + controls_wrapper_start + dispatch_html + controls_html + controls_wrapper_end + left_col_end

# Replace the old map section with the new left column
html = html.replace(map_html, new_left_col)

with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/index.html', 'w') as f:
    f.write(html)

print("HTML restructured successfully.")
