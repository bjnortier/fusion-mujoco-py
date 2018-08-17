import os
import adsk.core, adsk.fusion, traceback
import sys
import csv

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from fusion_mujoco_py.cohort import make_combinations
from fusion_mujoco_py.export import write_stls_and_mojoco_xml

def capture_screenshot(output_folder, instance_name):
    app = adsk.core.Application.get()
    app.activeViewport.fit()
    app.activeViewport.saveAsImageFile(os.path.join(output_folder, '{}.jpg'.format(instance_name)), 640, 640);

def set_render_workspace():
    app = adsk.core.Application.get()
    ui  = app.userInterface
    for workspace in ui.workspaces:
        if workspace.name == 'Render':
            workspace.activate()
            break

def set_camera():
    app = adsk.core.Application.get()
    cam = app.activeViewport.camera
    cam.eye = adsk.core.Point3D.create(100, -100, 20)
    cam.target = adsk.core.Point3D.create(0, 0, 0)
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    app.activeViewport.camera = cam
    app.activeViewport.fit()

def get_input_csv_filename(ui):
    csvInputDialog = ui.createFileDialog()
    csvInputDialog.isMultiSelectEnabled = False
    csvInputDialog.title = 'Input parameter ranges CSV file'
    csvInputDialog.filter = '*.csv'
    dialogResult = csvInputDialog.showOpen()
    if dialogResult == adsk.core.DialogResults.DialogOK:
        return csvInputDialog.filename
    else:
        return None

def parse_input_csv(filename):
    """
    CSV file has 4 lines
    1. User variable name
    2. Min value
    3. Max value
    4. Step value
    """
    file = open(filename)
    lines = file.readlines()
    assert(len(lines) == 4)
    vars  = list(map(lambda x: x.strip(), lines[0].split(',')))
    mins  = list(map(lambda x: x.strip(), lines[1].split(',')))
    maxs  = list(map(lambda x: x.strip(), lines[2].split(',')))
    steps = list(map(lambda x: x.strip(), lines[3].split(',')))
    return (vars, mins, maxs, steps)

def generate_all_values(vars, mins, maxs, steps):
    all_values = []
    for index in range(len(vars)):
        min = float(mins[index])
        max = float(maxs[index])
        step = float(steps[index])
        values = []
        value = min
        while value <= max:
            values.append(value)
            value += step
        all_values.append((vars[index], values))
    return all_values

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        # Read the parameter ranges from the input CSV file
        csv_input_filename = get_input_csv_filename(ui)
        if not csv_input_filename:
            return
        output_directory = os.path.dirname(csv_input_filename)
        print('output directory:', output_directory)
        # once-off viewport setup
        set_render_workspace()
        set_camera()

        (vars, mins, maxs, steps) = parse_input_csv(csv_input_filename)
        all_values = generate_all_values(vars, mins, maxs, steps)
        combinations = make_combinations(all_values)

        # For each parameter combination, update the Fusion model,
        # capture a screenshot, and export the Mujoco model (which includes the
        # STL files.
        # Write a output CSV file descriping each instance
        output_csv_filename = os.path.join(output_directory, 'instances.csv')
        with open(output_csv_filename, 'w', newline='\n') as output_csv_file:
            output_csv_writer = csv.writer(output_csv_file)
            for idx, combination in enumerate(combinations):
                # CSV Header. Combination have the same sequence
                # of variable names
                if (idx == 0):
                    header = list(map(lambda x: x[0], combination))
                    output_csv_writer.writerow(header)
                output_csv_writer.writerow(map(lambda x: x[1], combination))
                instance_name = str(idx)
                instance_output_directory = os.path.join(output_directory, instance_name)
                try:
                    os.mkdir(instance_output_directory)
                    os.mkdir(os.path.join(instance_output_directory, 'stl'))
                except FileExistsError as e:
                    print('overwriting existing STLs for instance: {}'.format(instance_name))
                for var, value in combination:
                    param = design.userParameters.itemByName(var)
                    param.expression = '{}mm'.format(value)
                capture_screenshot(instance_output_directory, instance_name)
                write_stls_and_mojoco_xml(design, instance_output_directory, instance_name)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        else:
            print('Failed:\n{}'.format(traceback.format_exc()))
