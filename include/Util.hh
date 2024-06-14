/*
   Copyright (C) 2022 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Export.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

/// \brief Helper function to get an entity given its unscoped name.
///
/// \param[in] _name Entity's unscoped name.
/// \param[in] _ecm Immutable reference to ECM.
/// \param[in] _relativeTo Entity that the unscoped name is relative to.
/// If not provided, the unscoped name could be relative to any entity.
/// \return All entities that match the unscoped name and relative to
/// requirements, or an empty set otherwise.
std::unordered_set<Entity> EntitiesFromUnscopedName(
    const std::string &_name, const EntityComponentManager &_ecm,
    Entity _relativeTo = kNullEntity);

/// \brief Get the ID of a joint entity which is a descendent of this model.
///
/// A replacement for gz::sim::Model::JointByName which does not resolve
/// joints for nested models.
/// \param[in] _ecm Entity-component manager.
/// \param[in] _entity Model entity.
/// \param[in] _name Scoped joint name.
/// \return Joint entity.
Entity JointByName(EntityComponentManager &_ecm,
    Entity _modelEntity,
    const std::string &_name);

}
}  // namespace sim
}  // namespace gz
