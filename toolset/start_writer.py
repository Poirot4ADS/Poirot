 
import subprocess
import time

def run_docker_command(user, container_name, command):
    cmd = f'bash -c "cd /apollo && {command}"'
    full_command = f'docker exec -u {user} {container_name} {cmd}'
    # full_command = f"docker exec {container_name} {command}"
    try:
        
        process = subprocess.Popen(
            full_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        
        time.sleep(3)

        
        if process.poll() is not None:
            stdout, stderr = process.communicate(timeout=1)
            return {
                "stdout": stdout.strip(),
                "stderr": stderr.strip(),
                "returncode": process.returncode
            }

        
        return {
            "stdout": "Docker 命令已启动",
            "stderr": "",
            "returncode": 0
        }

    except Exception as e:
        return {
            "stdout": "",
            "stderr": str(e),
            "returncode": 1
        }




