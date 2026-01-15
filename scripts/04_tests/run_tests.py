#!/usr/bin/env python3

import os
import subprocess
import time
import re
import signal
import sys
import yaml
from pathlib import Path

# --- Configuration ---

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
SRC_DIR = WORKSPACE_ROOT / "src"
CONFIG_DIR = SRC_DIR / "4_interfaces" / "lesson_interfaces" / "config"

# Track definitions
TRACKS = {
    "1_python": {"suffix": "_py", "executable_prefix": "lesson_"},
    "2_cpp": {"suffix": "_cpp", "executable_prefix": ""},
    "3_rust/1_rclrs": {"suffix": "_rclrs", "executable_prefix": ""},
    "3_rust/2_rcllibrust": {"suffix": "_rcllibrust", "executable_prefix": ""},
}

# Lesson definitions
LESSONS = [
    # Lesson 01: Simple Node
    {
        "id": "01",
        "name": "lesson_01_node",
        "type": "node",
        "timeout": 5,
        "verify_regex": r"started|tick",
    },
    # Lesson 02: Publisher
    {
        "id": "02",
        "name": "lesson_02_publisher",
        "type": "publisher",
        "topic": "/tutorial/chatter",
        "msg_type": "lesson_interfaces/msg/MsgCount",
    },
    # Lesson 03: Subscriber
    {
        "id": "03",
        "name": "lesson_03_subscriber",
        "type": "subscriber",
        "topic": "/tutorial/chatter",
        "msg_type": "lesson_interfaces/msg/MsgCount",
        "pub_data": "{count: 1}",
        "verify_regex": r"Received.*1|Detected counter reset.*1",
    },
    # Lesson 04: Service
    {
        "id": "04",
        "name": "lesson_04_service",
        "type": "service",
        "service_name": "/compute_stats",
        "service_type": "lesson_interfaces/srv/ComputeStats",
        "req_data": "{data: [1.0, 2.0, 3.0]}",
        "verify_regex": r"sum=6.0",
    },
    # Lesson 05: Parameters
    {
        "id": "05",
        "name": "lesson_05_parameters",
        "type": "parameters",
        # Publisher part
        "pub_name": "lesson_05_publisher",
        "sub_name": "lesson_05_subscriber",
        "param_file_topics": str(CONFIG_DIR / "topics_config.yaml"),
        "param_file_qos": str(CONFIG_DIR / "qos_config.yaml"),
        "param_file_services": str(CONFIG_DIR / "services_config.yaml"),
    }
]

# --- Helpers ---

def run_command_timeout(cmd, timeout_sec, verify_regex=None, cwd=WORKSPACE_ROOT):
    """Result: (success, output)"""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            shell=True,
            cwd=cwd,
            env=env,
            preexec_fn=os.setsid,
            universal_newlines=True
        )
        
        output = ""
        try:
            # Wait for command to finish or timeout
            # For persistent nodes (timeout expected), this raises TimeoutExpired
            # For quick commands, it returns (stdout, stderr)
            stdout, _ = proc.communicate(timeout=timeout_sec)
            output = stdout
            
            # If we get here, process exited naturally within timeout
            if verify_regex:
                if re.search(verify_regex, output):
                   return True, output
                return False, output
            return (proc.returncode == 0), output
            
        except subprocess.TimeoutExpired as e:
            # Process timed out (still running). Kill it.
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            # Get the partial output
            # communicate() usually sets stdout on the exception in newer Python; 
            # if not, we might lose it, but usually text mode works.
            # actually e.stdout is bytes or string.
            output = e.stdout if e.stdout else ""
            if isinstance(output, bytes):
                output = output.decode('utf-8', errors='replace')
                
            if not output:
                 # sometimes it might process
                 output = "[TIMEOUT - No Output Captured]"
            
            if verify_regex:
                if re.search(verify_regex, output):
                    return True, output
                return False, output + "\n[TIMEOUT - Regex Not Found]"
            
            return False, output + "\n[TIMEOUT]"
            
    except Exception as e:
        return False, str(e)


import tempfile

def run_background_node(cmd, cwd=WORKSPACE_ROOT):
    """Returns (Popen object, open file object for stdout)"""
    # Create a loopback temp file to store output so we can debug failures
    # We must keep the file open or it might be deleted if we use NamedTemporaryFile with delete=True (default)
    # But we want it to persist until we read it.
    
    out_file = tempfile.TemporaryFile(mode='w+')
    
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"

    proc = subprocess.Popen(
        cmd,
        stdout=out_file,
        stderr=out_file, # Combine for easier debug
        shell=True,
        cwd=cwd,
        env=env,
        preexec_fn=os.setsid
    )
    return proc, out_file

def kill_process(proc):
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=2)
        except:
            pass

# --- Test Logic ---

def test_node(pkg, exec_name, regex, timeout=5):
    cmd = f"ros2 run {pkg} {exec_name}"
    success, output = run_command_timeout(cmd, timeout, regex)
    return success, output

def test_publisher(pkg, exec_name, topic, msg_type):
    # Start the publisher in background
    pub_cmd = f"ros2 run {pkg} {exec_name}"
    pub_proc, pub_out = run_background_node(pub_cmd)
    time.sleep(2) # Warmup
    
    # Listen with ros2 topic echo
    echo_cmd = f"ros2 topic echo {topic} {msg_type} --once"
    success, output = run_command_timeout(echo_cmd, 5)
    
    kill_process(pub_proc)
    pub_out.close() # Clean up temp file
    return success, output

def test_subscriber(pkg, exec_name, topic, msg_type, pub_data, regex):
    # Start subscriber
    sub_cmd = f"ros2 run {pkg} {exec_name}"
    sub_proc, sub_out = run_background_node(sub_cmd)
    time.sleep(2) # Warmup
    
    # Publish once
    pub_cmd = f"ros2 topic pub {topic} {msg_type} \"{pub_data}\" --once"
    run_command_timeout(pub_cmd, 3)
    
    # We need to capture the SUBSCRIBER's output to verify receipt.
    # Ah, run_background_node silences output. We need to actually read it.
    # Modified approach: Run subscriber with timeout, monitoring line by line.
    
    kill_process(sub_proc) 
    sub_out.close() 
    
    # Real approach: Run subscriber, wait for regex match or timeout. 
    # In parallel, fire the publisher after a short delay.
    
    # Since we operate strictly sequentially here, we have to cheat:
    # 1. Start subscriber, capturing stdout
    # 2. In a separate thread/process, publish.
    # OR: Just run the subscriber for N seconds, and publish in the middle.
    
    cmd = f"ros2 run {pkg} {exec_name}"
    
    # Create a publisher trigger
    def trigger_pub():
        time.sleep(2)
        # Publish multiple times to ensure discovery (Best Effort / Volatile can miss the first one)
        for _ in range(3):
            subprocess.run(
                f"ros2 topic pub {topic} {msg_type} \"{pub_data}\" --once", 
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            time.sleep(0.5)
        
    import threading
    t = threading.Thread(target=trigger_pub)
    t.start()
    
    success, output = run_command_timeout(cmd, 6, regex)
    t.join()
    return success, output

def test_service(pkg, exec_name, srv_name, srv_type, req_data, regex):
    # Start server
    srv_cmd = f"ros2 run {pkg} {exec_name}"
    srv_proc, srv_out = run_background_node(srv_cmd)
    time.sleep(2)
    
    # Call service
    call_cmd = f"ros2 service call {srv_name} {srv_type} \"{req_data}\""
    success, output = run_command_timeout(call_cmd, 5, regex)
    
    if not success:
         # Capture server output for debug
         srv_out.seek(0)
         srv_log = srv_out.read()
         output += f"\n[SERVER LOG]:\n{srv_log}"
    
    kill_process(srv_proc)
    srv_out.close()
    return success, output

def test_parameters_publisher(pkg, exec_name, lesson_cfg):
    # Test valid configuration loading
    # Warning: Param files need --params-file
    cmd = f"ros2 run {pkg} {exec_name} --ros-args --params-file {lesson_cfg['param_file_topics']} --params-file {lesson_cfg['param_file_qos']} --params-file {lesson_cfg['param_file_services']}"
    
    # We want to verify it picked up the "timer_period_s" from defaults or config?
    # Actually, config doesn't set timer_period_s usually (it's in code default or passed on CLI).
    # But it DOES set topic names.
    # Let's verify the topic name is NOT "chatter" (default) but what's in the YAML.
    # Wait, the YAML probably says "chatter" or similar.
    # Let's check the YAML content first... assume it's "chatter" or "tutorial/telemetry".
    # For now, let's just ensure it runs and we can get a param.
    
    proc, proc_out = run_background_node(cmd)
    time.sleep(2)
    
    # Determine node name (might differ from exec name, e.g. C++ target vs code)
    node_name = lesson_cfg.get("node_name", exec_name)

    # Check if param exists
    check_cmd = f"ros2 param get /{node_name} timer_period_s"
    success, output = run_command_timeout(check_cmd, 3, r"Double|Integer") # Should return value
    
    if not success:
         # Capture output
         proc_out.seek(0)
         node_log = proc_out.read()
         output += f"\n[NODE LOG]:\n{node_log}"

    kill_process(proc)
    proc_out.close()
    return success, output

# --- Main Test Loop ---

def main():
    print(f"Running tests in {WORKSPACE_ROOT}")
    
    results = {}
    
    for track_path, track_info in TRACKS.items():
        track_name = track_path.replace("/", "_")
        results[track_name] = {}
        
        print(f"\n=== Testing Track: {track_path} ===")
        
        for lesson in LESSONS:
            lid = lesson["id"]
            lname = lesson["name"] # Base name e.g. lesson_01_node
            
            # Construct package name
            # e.g. lesson_01_node_py
            pkg_name = f"{lname}{track_info['suffix']}"
            
            # Construct executable name
            # Python: lesson_01_node
            # C++: node
            # Rust: node matches package? No, often just "node".
            # We need to be careful.
            # Python Lesson 01: entry_point='lesson_01_node = ...'
            # C++ Lesson 01: exec 'node'
            # Rust Lesson 01: bin 'lesson_01_node' ?? Check CMakeLists/Cargo.toml
            
            # Heuristic adjustment for executables
            exec_name = "node" # Default for C++ (L01-L03)
            
            if "python" in track_path:
                 exec_name = lname
            elif "cpp" in track_path:
                 if lesson["id"] == "01": exec_name = "node"
                 if lesson["id"] == "02": exec_name = "node"
                 if lesson["id"] == "03": exec_name = "node"
                 if lesson["id"] == "04": exec_name = "service_server"
                 if lesson["id"] == "05": exec_name = "lesson_05_publisher_cpp" # Checked
            else:
                 # Rust
                 exec_name = lname # Default guess
                 if lesson["id"] == "01": exec_name = "lesson_01_node_rclrs" # specific to rclrs
                 if lesson["id"] == "02": exec_name = "lesson_02_node"
                 if lesson["id"] == "03": exec_name = "lesson_03_subscriber_rclrs" # Fixed per manual verification
                 if lesson["id"] == "04": exec_name = "service_server"
                 if lesson["id"] == "05": exec_name = "lesson_05_publisher"

            if "rcllibrust" in track_path:
                 # rcllibrust might have different names, assume standard for now or same as rclrs
                 pass

            if lesson["id"] == "05":
                # Ensure we check the correct node name for params
                lesson["node_name"] = "lesson_05_publisher"
                if "python" in track_path:
                    exec_name = "lesson_05_publisher"

            # Check if package exists
            # simplest way: `ros2 pkg prefix pkg_name`
            check_pkg, _ = run_command_timeout(f"ros2 pkg prefix {pkg_name}", 2)
            if not check_pkg:
                print(f"  [SKIP] {lid} {pkg_name} not found")
                results[track_name][lid] = "SKIP"
                continue
            
            print(f"  [TEST] {lid} {pkg_name} / {exec_name} ... ", end="", flush=True)
            
            passed = False
            output = "No Output"
            try:
                if lesson["type"] == "node":
                    passed, output = test_node(pkg_name, exec_name, lesson["verify_regex"], lesson["timeout"])
                elif lesson["type"] == "publisher":
                    passed, output = test_publisher(pkg_name, exec_name, lesson["topic"], lesson["msg_type"])
                elif lesson["type"] == "subscriber":
                    passed, output = test_subscriber(pkg_name, exec_name, lesson["topic"], lesson["msg_type"], lesson["pub_data"], lesson["verify_regex"])
                elif lesson["type"] == "service":
                    # For Service lesson, standard exec name might differ
                    if "python" in track_path:
                        srv_exec = "service_server"
                    elif "cpp" in track_path:
                        srv_exec = "service_server"
                    else:
                        srv_exec = "service_server" # Rust guess
                        
                    passed, output = test_service(pkg_name, srv_exec, lesson["service_name"], lesson["service_type"], lesson["req_data"], lesson["verify_regex"])
                elif lesson["type"] == "parameters":
                    # Test Publisher
                    passed, output = test_parameters_publisher(pkg_name, exec_name, lesson)
                    
            except Exception as e:
                print(f"ERROR: {e}")
                output = str(e)
                passed = False
                
            if passed:
                print("PASS")
                results[track_name][lid] = "PASS"
            else:
                print("FAIL")
                print(f"    >>> Output:\n{output}\n    <<<")
                results[track_name][lid] = "FAIL"

    # print summary
    print("\n\n" + "="*40)
    print("       LESSON SUMMARY")
    print("="*40)
    
    header = f"{'Track':<20} | {'01':^6} | {'02':^6} | {'03':^6} | {'04':^6} | {'05':^6}"
    print(header)
    print("-" * len(header))
    
    for track, data in results.items():
        row = f"{track:<20} |"
        for i in range(1, 6):
            lid = f"{i:02d}"
            res = data.get(lid, " -- ")
            row += f" {res:^6} |"
        print(row)
    print("="*40)

if __name__ == "__main__":
    main()
