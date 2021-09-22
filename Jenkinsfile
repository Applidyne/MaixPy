pipeline {
    agent any
    options {
            timeout(time: 120, unit: 'MINUTES')
    }
    stages {
        stage('Build') {
            steps {
                    echo '******* MaixPy Build *******'
                    bat '''
                        cd projects/maixpy_k210_minimum
                        dir C:\\Windows\\Sysnative
                        cmd -c C:\\Windows\\Sysnative\\wsl.exe --help"
                        cmd -c C:\\Windows\\Sysnative\\wsl.exe echo "Hello World!"
                        cmd -c C:\\Windows\\Sysnative\\wsl.exe python3 project.py build
                        cmd -c C:\\Windows\\Sysnative\\wsl.exe echo "Done!"
                    '''
                  }
            post {
                always {
                    archiveArtifacts artifacts: 'projects/maixpy_k210_minimum/build/maixpy*'
                }
            }
        }
    }
}
