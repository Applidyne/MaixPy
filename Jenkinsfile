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
                        cd projects/hello_world
                        C:\\Windows\\Sysnative\\wsl.exe python3 project.py build
                    '''
                  }
            post {
                always {
                    archiveArtifacts artifacts: 'projects/hello_world/build/hello_word*'
                }
            }
        }
    }
}
