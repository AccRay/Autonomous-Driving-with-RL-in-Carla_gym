import collections
import logging
import weakref,math
import carla


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class CollisionSensor(object):
    """Class for collision sensors"""
    def __init__(self,parent_actor):
        self.sensor=None
        self.history=[]
        self._parent=parent_actor
        world=self._parent.get_world()
        blueprint=world.get_blueprint_library().find('sensor.other.collision')
        self.sensor=world.spawn_actor(blueprint,carla.Transform(),attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_ref=weakref.ref(self)
        self.sensor.listen(lambda event:CollisionSensor._on_collision(weak_ref,event))

    def get_collision_history(self):
        """Get the histroy of collisions"""
        history=collections.defaultdict(int)
        for frame,intensity in self.history:
            history[frame]+=intensity
        return history

    @staticmethod
    def _on_collision(weak_ref,event):
        """On collision method"""
        self=weak_ref()
        if not self:
            return
        actor_type=get_actor_display_name(event.other_actor)
        logging.info('Collision with %r',actor_type)
        impulse=event.normal_impulse
        intensity=math.sqrt(impulse.x**2+impulse.y**2+impulse.z**2)
        self.history.append((event.frame,intensity))
        if len(self.history)>4000:
            self.history.pop(0)