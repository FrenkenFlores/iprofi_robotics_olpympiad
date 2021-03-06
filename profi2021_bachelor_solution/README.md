#### Важные замечания

* Желаемая высота полета квадрокоптера в различных тестовых сценариях задается посредством рос-параметра **altitude_desired** в момент запуска рос-пакета. Участник должен реализовать соответствующий функционал.

* Движение квадрокоптера по трассе должно осуществляться по часовой стрелке. Модель квадрокоптера создается в сцене так, что "перед" квадрокоптера направлен в том направлении трассы, куда должно быть начато движение. (Т.е. при создании модели квадрокоптера в сцене ось X системы координат `base_link` квадрокоптера направлена в сторону движения по трассе по часовой стрелке)

* При реализации алгоритмов управления рекомендуется использоваться время симуляции `rostime`

* Решение участника должно запскаться вызовом единствевнного ros-launch файла `launch/solution.launch`. 


![quadrator_track_1](docs/pics/quadrator_track_1.png)

Репозиторий содержит программное обеспечение для
подготовки и сдачи задания категории _Бакалавриат_  направления Робототехники олимпиады [Я - Профессионал](https://yandex.ru/profi/courses2020).

Данный репозиторий будет использоваться для проведения оценки заданий участников.

Предоставялемое программное обеспечение включает ROS-пакет, содержащий пример управления квадрокоптером и получения изображения с его бортовой камеры в симуляторе gazebo.

Работа с пакетом предполагается из docker-контейнера предоставляемого участникам в составе [основного репозитория олимпиады](https://gitlab.com/beerlab/iprofi2021/profi2021_robotics) и совместно с [репозиторием с описанием сцены и квадрокоптера для симулятора](https://gitlab.com/beerlab/iprofi2021/profi2021_bachelor_scene), соответственно, предоставляемые далее инструкции требуют предварительно выполнить инструкции, представленные в обозначенных репозиториях.


#### Установка программного обеспечения

1. В новом терминале **вне docker-контейнера** перейдите в корневую директорию основного репозитория олимпиады и склонируйте данный репозиторий в рабочее окружение выполнив:

        cd workspace/src
        git clone https://gitlab.com/beerlab/iprofi2021/profi2021_bachelor_solution.git

2. Откройте bash-сессию **внутри контейнера**, перейдите в смонтированное рабочее окружение и соберите все пакеты:

        bash docker/into_docker.sh
        cd /workspace
        catkin build


#### Описание программного обеспечения

1. Репозиторий представляет собой ROS-пакет, который участникам олимпиады предстоит использовать для представления своего решения для оценки. Пакет включает в себя пример управления квадрокоптером и получения изображения с его бортовой камеры.

2. Перед запуском примера убедитесь, что программное обеспечение из [репозитория с описанием сцены и квадрокоптера для симулятора](https://gitlab.com/beerlab/iprofi2021/profi2021_bachelor_scene) установленно корректно.

    Запустите симулятор выполнив:

        roslaunch profi2021_bachelor_scene start_scene.launch

    Дождитесь окончания загрузки симулятора и появления квадрокоптера на сцене. Для запуска примера в новой bash-сессии **внутри контейнера** выполните:

        roslaunch profi2021_bachelor_solution solution.launch

    При успешном выполнении всех выше обозначенных действий должно появиться дополнительное окно с выводом изображения с бортовой камеры квадрокоптера, сам квадрокоптер осуществит взлет на некоторую высоту и начнет исполнять траекторию в форме восьмерки:

    ![quadrator_track_1_example](docs/pics/quadrator_track_1_example.gif)

    Исходный код примера находится в *scripts/simple_move.py* (*src/simple_move.cpp*)  и представляет собой ROS-узел */quadrator_simple_move*. Узел подписывается на [топик с изображением */image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) с камеры робота, запускает двигатели квадрокоптера используя [сервис */enable_motors*](https://gitlab.com/beerlab/iprofi2021/profi2021_bachelor_scene/-/blob/master/hector_uav_msgs/srv/EnableMotors.srv) и публикует желаемую скорость робота в [топик */cmd_vel*](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html). На рисунке ниже соответствующие топики и узлы представленны схематически:

    ![scheme](docs/pics/scheme.png)
