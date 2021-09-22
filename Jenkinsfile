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
                        C:\\Windows\\Sysnative\\wsl.exe echo "Hello World!"
                        C:\\Windows\\Sysnative\\wsl.exe python3 project.py build
                        C:\\Windows\\Sysnative\\wsl.exe echo "Done!"
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
