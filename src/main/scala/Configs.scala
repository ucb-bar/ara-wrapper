package ara

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import shuttle.common.{ShuttleTileAttachParams, ShuttleCoreVectorParams}

class WithAraRocketVectorUnit(vLen: Int = 4096, nLanes: Int = 2, axiIdBits: Int = 4, cores: Option[Seq[Int]] = None, enableDelay: Boolean = false) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      val buildVector = cores.map(_.contains(tp.tileParams.tileId)).getOrElse(true)
      require(nLanes >= 2)
      if (buildVector) tp.copy(tileParams = tp.tileParams.copy(
        core = tp.tileParams.core.copy(
          vector = Some(RocketCoreVectorParams(
            build = ((p: Parameters) => new AraRocketUnit(nLanes, axiIdBits, enableDelay)(p)),
            vLen = vLen,
            eLen = 64,
            vfLen = 64,
            vfh = false,
            vMemDataBits = 0,
            decoder = ((p: Parameters) => {
              val decoder = Module(new AraEarlyVectorDecode()(p))
              decoder
            }),
            useDCache = false,
            issueVConfig = true,
            vExts = Nil
          )),
        )
      )) else tp
    }
    case other => other
  }
})

class WithAraShuttleVectorUnit(vLen: Int = 4096, nLanes: Int = 2, axiIdBits: Int = 4, cores: Option[Seq[Int]] = None, enableDelay: Boolean = false) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => {
      val buildVector = cores.map(_.contains(tp.tileParams.tileId)).getOrElse(true)
      if (buildVector) tp.copy(tileParams = tp.tileParams.copy(
        core = tp.tileParams.core.copy(
          vector = Some(ShuttleCoreVectorParams(
            build = ((p: Parameters) => new AraShuttleUnit(nLanes, axiIdBits, enableDelay)(p)),
            vLen = vLen,
            vfLen = 64,
            vfh = false,
            decoder = ((p: Parameters) => {
              val decoder = Module(new AraEarlyVectorDecode()(p))
              decoder
            }),
            issueVConfig = true,
            vExts = Nil
          )),
        )
      )) else tp
    }
    case other => other
  }
})
