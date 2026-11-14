"""Gazebo collision filter groups (SDF <category_bitmask> / <collide_bitmask>).

Bullet merges simulation islands on broadphase AABB overlap, not on contact, and its
dynamic tree inflates AABBs by ~5 cm. Board pieces ~40 mm apart therefore chained into
one island, and whenever the always-active arm came near the board (the observation /
home pose is ~3-6 cm from the first row) all 32 pieces were kept awake and solved
with the robot every 0.5 ms step. Measured 2026-11-14 at the home pose:
RTF 0.30 (camera ~9 FPS) -> 0.97 with the robot filtered from pieces; removing the
robot-side half of the pieces gave the same 0.97.

Groups below are valid under both the SDF rule (OR of the two cross terms) and
Bullet's (AND of the two cross terms):

              piece  board/table  arm(base..link_5)  gripper+link_6 (default)
  piece         no       yes            no                  yes
  board/table   yes      -              yes                 yes
  arm           no       yes            -                   -

Simplifications: pieces do not collide with each other, and the upper/lower arm does
not touch pieces. Fingers, gripper body and wrist (link_6) still contact pieces, and
pieces still rest on the board. On a normal board pieces never touch each other; a
piece dropped onto another would overlap instead of tumbling.
"""

PIECE_CATEGORY = 0x01
ENVIRONMENT_CATEGORY = 0x02
ARM_CATEGORY = 0x04

PIECE_COLLIDE = ENVIRONMENT_CATEGORY
ENVIRONMENT_COLLIDE = PIECE_CATEGORY | ARM_CATEGORY
ARM_COLLIDE = ENVIRONMENT_CATEGORY

ARM_LINKS = frozenset({'base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5'})


def collides(category_a, collide_a, category_b, collide_b):
    """True under both the SDF (OR) and Bullet (AND) filter rules; used by tests."""
    sdf = bool((category_a & collide_b) | (category_b & collide_a))
    bullet = bool(category_a & collide_b) and bool(category_b & collide_a)
    if sdf != bullet:
        raise ValueError('Filter groups are ambiguous between SDF and Bullet rules')
    return sdf


def contact_xml(category, collide):
    """SDF <contact> element; decimal because SDF parses unsigned ints."""
    return (f'<contact><collide_bitmask>{int(collide)}</collide_bitmask>'
            f'<category_bitmask>{int(category)}</category_bitmask></contact>')
