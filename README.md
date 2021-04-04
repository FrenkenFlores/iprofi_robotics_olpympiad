### Директория для размещения ROS-пакетов

* Сборка пакетов происходит внутри docker-контейнера:

        bash docker/into_docker.sh
        cd /workspace
        catkin build

* Для сборки пакетов индивидуально можно воспользоваться:

         catkin build <my_package>

* За дополнительной информацией можно обратиться к [оффициальной документации catkin_tools](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html).

* Использовать в одном рабочем окружении catkin_tools и catkin_make одновременно нельзя.

* Для разработки рекомендуется использовать любой текстовый редактор на хосте. Соответственно, внутри docker-контейнера проводить только сборку и тестирование. 
# iprofi_robotics_olpympiad
