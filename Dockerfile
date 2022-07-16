FROM giordanolaminetti/slampy:orbslam2-ros-melodic
USER root
RUN chown -R slampy /slampy/slampy
RUN chmod -R 755 /slampy
USER slampy
RUN pip3 install jupyterlab pycryptodomex gnupg rospkg scipy
