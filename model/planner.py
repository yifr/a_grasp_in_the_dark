import os
import numpy as np
from pydrake.all import (
    LeafSystem,
    Context,
    Meshcat,
    State,
    ContactResults,
    RigidTransform,
    RotationMatrix,
    AbstractValue
)
from manipulation.meshcat_utils import AddMeshcatTriad


class Planner(LeafSystem):
    """Planner maintains a list of contact points. 
    When called, it queries current contact points, decides on a new target location,
    and returns a trajectory to the new target location.
    """

    def __init__(
        self,
        time_step=0.1,
        meshcat=None,
    ):
        super().__init__()
        self._meshcat = meshcat
        self._contact_points = set()    # list of contact points in world coordinates

        self.DeclareAbstractInputPort("contact_results", AbstractValue.Make(ContactResults()))
        self.DeclarePeriodicDiscreteUpdateEvent(0.01, 0, self._update_contact_points)

    
    def _update_contact_points(self, context, state):
        contact_results = self.EvalAbstractInput(context, 0).get_value()
        self._contact_points = set()
        for contact_info in contact_results.contact_info:
            p_WC = contact_info.point
            self._contact_points.add(p_WC)

        return 