package ara

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._

class WithAraRocketVectorUnit(nLanes: Int = 2, axiIdBits: Int = 4, cores: Option[Seq[Int]] = None) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      val buildVector = cores.map(_.contains(tp.tileParams.tileId)).getOrElse(true)
      require(nLanes >= 2)
      if (buildVector) tp.copy(tileParams = tp.tileParams.copy(
        core = tp.tileParams.core.copy(
          vector = Some(RocketCoreVectorParams(
            build = ((p: Parameters) => new AraRocketUnit(nLanes, axiIdBits)(p)),
            vLen = 4096,
            vMemDataBits = 0,
            decoder = ((p: Parameters) => {
              val decoder = Module(new AraEarlyVectorDecode()(p))
              decoder
            }),
            useDCache = false,
            issueVConfig = true
          )),
        )
      )) else tp
    }
    case other => other
  }
})