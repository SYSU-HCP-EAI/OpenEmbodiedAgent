def dump_extensions():

    extensions = {}
    from internutopia.core.robot.controller import BaseController

    extensions['controllers'] = BaseController.controllers
    from internutopia.core.util.interaction import BaseInteraction

    extensions['interactions'] = BaseInteraction.interactions
    from internutopia.core.task.metric import BaseMetric

    extensions['metrics'] = BaseMetric.metrics
    from internutopia.core.object.object import BaseObject

    extensions['objs'] = BaseObject.objs
    from internutopia.core.robot.robot import BaseRobot

    extensions['robots'] = BaseRobot.robots
    from internutopia.core.sensor.sensor import BaseSensor

    extensions['sensors'] = BaseSensor.sensors
    from internutopia.core.task import BaseTask

    extensions['tasks'] = BaseTask.tasks
    return extensions


def reload_extensions(extensions):

    from internutopia.core.robot.controller import BaseController

    BaseController.controllers = extensions['controllers']
    from internutopia.core.util.interaction import BaseInteraction

    BaseInteraction.interactions = extensions['interactions']
    from internutopia.core.task.metric import BaseMetric

    BaseMetric.metrics = extensions['metrics']
    from internutopia.core.object.object import BaseObject

    BaseObject.objs = extensions['objs']
    from internutopia.core.robot.robot import BaseRobot

    BaseRobot.robots = extensions['robots']
    from internutopia.core.sensor.sensor import BaseSensor

    BaseSensor.sensors = extensions['sensors']
    from internutopia.core.task import BaseTask

    BaseTask.tasks = extensions['tasks']
