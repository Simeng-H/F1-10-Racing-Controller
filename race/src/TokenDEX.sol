// SPDX-License-Identifier: MIT
// Quentin Bishop qjb8jg

pragma solidity ^0.8.7;

import "./DEX.sol";
import "./IERC20.sol";
import "./IERC20Metadata.sol";

contract TokenDEX is DEX{

    address private contractOwner;
    address private etherPricerContract;
    address private erc20tokenContract;
    uint public override k;
    uint public override etherLiquidity;
    uint public override tokenLiquidity;
    uint public override feeNumerator;
    uint public override feeDenominator;
    uint public override feesEther;
    uint public override feesToken;

    
    mapping (address => uint) public override etherLiquidityForAddress;
    mapping (address => uint) public override tokenLiquidityForAddress;

    constructor () {
        contractOwner = msg.sender;
    }

    function createPool(uint tokenAmount, uint feeNum, uint feeDenom, address erc20token, address etherpricer) external payable override{
        require(msg.sender==contractOwner,"Only deployer of contract can call createPool");
        etherLiquidity = msg.value;
        tokenLiquidity = tokenAmount;
        feeNumerator = feeNum;
        feeDenominator = feeDenom;
        erc20tokenContract = erc20token;
        etherPricerContract = etherpricer;
        k = etherLiquidity * tokenLiquidity;
        IERC20(erc20tokenContract).transferFrom(msg.sender,address(this),tokenAmount);
        emit liquidityChangeEvent();
    }

    function tokenDecimals() external view override returns (uint){
        return uint(IERC20Metadata(erc20tokenContract).decimals());
    }

    function getEtherPrice() external view override returns (uint){
        return EtherPricer(etherPricerContract).getEtherPriceInCents();
    }

    function getTokenPrice() external view override returns (uint){
        return etherLiquidity / tokenLiquidity * EtherPricer(etherPricerContract).getEtherPriceInCents();
    }

    function getPoolLiquidityInUSDCents() external view override returns(uint){
        return 2 * etherLiquidity * EtherPricer(etherPricerContract).getEtherPriceInCents();
    }


    function addLiquidity() external payable override {
        uint etherAmt = msg.value;
        uint tknAmt = etherAmt * tokenLiquidity / etherLiquidity;
        IERC20(erc20tokenContract).transferFrom(msg.sender, address(this),tknAmt);
        etherLiquidity += etherAmt;
        tokenLiquidity += tknAmt;
        k = tokenLiquidity * etherLiquidity;
        etherLiquidityForAddress[msg.sender] += etherAmt;
        tokenLiquidityForAddress[msg.sender] += tknAmt;
        emit liquidityChangeEvent();
    }

    function removeLiquidity(uint amountEther) external override {
        require(etherLiquidity > amountEther, "Can't remove more ether than exist in the pool");
        (bool success, ) = msg.sender.call{value: amountEther}("");
        require(success, "Failed to send Ether");
        uint tknAmt = amountEther * tokenLiquidity / etherLiquidity;
        IERC20(erc20tokenContract).transferFrom(address(this),erc20tokenContract,tknAmt);
        etherLiquidity -= amountEther;
        tokenLiquidity -= tknAmt;
        k = tokenLiquidity * etherLiquidity;
        etherLiquidityForAddress[msg.sender] += amountEther;
        tokenLiquidityForAddress[msg.sender] += tknAmt;
        emit liquidityChangeEvent();
    }

    function exchangeEtherForToken() external payable override{
        uint etherAmt = msg.value;
        address sender = msg.sender;
        uint tknAmt = tokenLiquidity - (k / (etherLiquidity+etherAmt));
        uint fee = feeNumerator * tknAmt / feeDenominator;
        feesToken += fee;
        //uint tknAmt = etherAmt * tokenLiquidity / etherLiquidity;
        IERC20(erc20tokenContract).transfer(sender,tknAmt-fee);
        etherLiquidity += etherAmt;
        tokenLiquidity -= tknAmt;
        k = tokenLiquidity * etherLiquidity;
        emit liquidityChangeEvent();
    }

    function exchangeTokenForEther(uint amountToken) external override{
        address sender = msg.sender;
        uint etherAmt = etherLiquidity - (k / (tokenLiquidity+amountToken));
        uint fee = feeNumerator * etherAmt / feeDenominator;
        feesEther += fee;
        //uint etherAmt = amountToken * etherLiquidity / tokenLiquidity;
        IERC20(erc20tokenContract).transfer(address(this),amountToken);
        uint val = etherAmt - fee;
        (bool success, ) = payable(sender).call{value: val}("");
        require(success, "Failed to send Ether");
        etherLiquidity -= etherAmt;
        tokenLiquidity += amountToken;
        k = tokenLiquidity * etherLiquidity;
        emit liquidityChangeEvent();
    }

    function setEtherPricer(address p) external override{
        etherPricerContract = p;
    }

    function etherPricerAddress() external override returns (address){
        return etherPricerContract;
    }

    function erc20TokenAddress() external override returns (address){
        return erc20tokenContract;
    }

    function getTokenCCAbbreviation() external override returns (string memory){
        return IERC20Metadata(erc20tokenContract).symbol();
    }

    function getDEXinfo() external override returns (address, string memory, string memory, address, uint, uint, uint, uint, uint, uint, uint, uint){
        return (address(this),IERC20Metadata(erc20tokenContract).symbol(), IERC20Metadata(erc20tokenContract).name(), erc20tokenContract, k, etherLiquidity, tokenLiquidity, feeNumerator, feeDenominator, IERC20Metadata(erc20tokenContract).decimals(), feesEther, feesToken);
    }

    function supportsInterface(bytes4 interfaceId) external view override returns (bool){
        return interfaceId == type(IERC20).interfaceId ||
               interfaceId == type(IERC165).interfaceId ||
               interfaceId == type(IERC20Metadata).interfaceId;
    }
}